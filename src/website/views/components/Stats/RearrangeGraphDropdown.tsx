'use client';

import { Fragment, useState, useEffect, useRef } from 'react';
import RearrangeIcon from '@/public/icons/format_line_spacing.svg';
import {
  DndContext,
  DragOverlay,
  useSensor,
  useSensors,
  PointerSensor,
  useDraggable,
} from '@dnd-kit/core';
import DragIndicatorIcon from '@/public/icons/drag_indicator.svg';
import styles from './stats.module.css';
import GraphsActions from '@/stores/Graphs/GraphsActions';
import { connect } from 'react-redux';
import { Layout, LayoutItem, GraphId, isSplitGroup } from '@/stores/Graphs/GraphsTypes';
import { moveGraphToIndex, splitGraph, findLayoutIndex } from '@/stores/Graphs/GraphsLayoutHelpers';

const graphsOrderNamesMap: Record<GraphId, string> = {
  GPS: 'Speed',
  BatteriesVoltage: 'Batteries Voltage',
  BatteriesCurrent: 'Batteries Current',
  WindSensors: 'Wind Sensors',
};

const getItemId = (item: LayoutItem): string => {
  return isSplitGroup(item) ? item[0] : item;
};

const getItemLabel = (item: LayoutItem): string => {
  if (isSplitGroup(item)) {
    return item.map((id) => graphsOrderNamesMap[id]).join(' | ');
  }
  return graphsOrderNamesMap[item];
};

const DraggableItem = ({
  id,
  label,
  isDragging,
  isSplitTarget,
}: {
  id: string;
  label: string;
  isDragging?: boolean;
  isSplitTarget?: boolean;
}) => {
  const { attributes, listeners, setNodeRef } = useDraggable({ id });

  const style = {
    opacity: isDragging ? 0.35 : 1,
    cursor: 'grab',
  };

  const className = isSplitTarget
    ? `${styles.dropdownItem} ${styles.dropdownItemSplitTarget}`
    : styles.dropdownItem;

  return (
    <div
      className={className}
      ref={setNodeRef}
      style={style}
      data-sortable-id={id}
      {...attributes}
      {...listeners}
    >
      <DragIndicatorIcon />
      {label}
    </div>
  );
};

const DropGap = ({
  index,
  isActive,
}: {
  index: number;
  isActive: boolean;
}) => {
  const className = isActive
    ? `${styles.dropGap} ${styles.dropGapActive}`
    : styles.dropGap;

  return <div className={className} data-drop-gap={index} />;
};

const RearrangeGraphDropdown = ({ graphs, rearrangeGraphs }: any) => {
  const [isOpen, setIsOpen] = useState(false);
  const [layout, setLayout] = useState<Layout>(graphs.layout);
  const [activeId, setActiveId] = useState<string | null>(null);
  const [splitTargetId, setSplitTargetId] = useState<string | null>(null);
  const [dropGapIndex, setDropGapIndex] = useState<number | null>(null);
  const dropdownRef = useRef<HTMLDivElement>(null);
  const hoverTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const hoverTargetRef = useRef<string | null>(null);
  const splitTargetRef = useRef<string | null>(null);
  const dropGapRef = useRef<number | null>(null);

  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (
        dropdownRef.current &&
        !dropdownRef.current.contains(event.target as Node)
      ) {
        setIsOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  const handleClick = () => setIsOpen(!isOpen);

  const clearHoverTimer = () => {
    if (hoverTimerRef.current) {
      clearTimeout(hoverTimerRef.current);
      hoverTimerRef.current = null;
    }
    hoverTargetRef.current = null;
  };

  const updateSplitTarget = (id: string | null) => {
    splitTargetRef.current = id;
    setSplitTargetId(id);
  };

  const updateDropGap = (index: number | null) => {
    dropGapRef.current = index;
    setDropGapIndex(index);
  };

  const onDragStart = (event: any) => {
    clearHoverTimer();
    updateSplitTarget(null);
    updateDropGap(null);
    setActiveId(event.active.id);
  };

  useEffect(() => {
    rearrangeGraphs(layout);
  }, [layout]);

  // Pointermove listener for split/gap detection (active during drag)
  useEffect(() => {
    if (!activeId) return;

    const sourceIndex = findLayoutIndex(layout, activeId as GraphId);

    const handlePointerMove = (e: PointerEvent) => {
      // Check items via elementsFromPoint (merge/split intent)
      const elements = document.elementsFromPoint(e.clientX, e.clientY);
      for (const el of elements) {
        const sortableId = (el as HTMLElement).dataset?.sortableId;
        if (sortableId && sortableId !== activeId) {
          updateDropGap(null);

          if (sortableId !== hoverTargetRef.current) {
            clearHoverTimer();
            updateSplitTarget(null);
            hoverTargetRef.current = sortableId;
            hoverTimerRef.current = setTimeout(() => {
              updateSplitTarget(sortableId);
            }, 500);
          }
          return;
        }
      }

      // Check gaps by bounding rect (reorder intent)
      // elementsFromPoint misses gaps because DragOverlay's portal sits on top
      const gapEls = Array.from(document.querySelectorAll<HTMLElement>('[data-drop-gap]'));
      for (const gapEl of gapEls) {
        const rect = gapEl.getBoundingClientRect();
        if (
          e.clientX >= rect.left &&
          e.clientX <= rect.right &&
          e.clientY >= rect.top &&
          e.clientY <= rect.bottom
        ) {
          const gapIndex = parseInt(gapEl.dataset.dropGap!, 10);

          // Skip gaps adjacent to the dragged item (no-op positions)
          if (gapIndex === sourceIndex || gapIndex === sourceIndex + 1) {
            clearHoverTimer();
            updateSplitTarget(null);
            updateDropGap(null);
            return;
          }

          clearHoverTimer();
          updateSplitTarget(null);
          updateDropGap(gapIndex);
          return;
        }
      }

      // Pointer is not over any item or gap → clear both
      clearHoverTimer();
      updateSplitTarget(null);
      updateDropGap(null);
    };

    document.addEventListener('pointermove', handlePointerMove);
    return () => {
      document.removeEventListener('pointermove', handlePointerMove);
      clearHoverTimer();
      updateSplitTarget(null);
      updateDropGap(null);
    };
  }, [activeId, layout]);

  const onDragEnd = (event: any) => {
    const { active } = event;
    const currentSplitTarget = splitTargetRef.current;
    const currentDropGap = dropGapRef.current;

    clearHoverTimer();
    updateSplitTarget(null);
    updateDropGap(null);
    setActiveId(null);

    if (currentSplitTarget) {
      setLayout(splitGraph(layout, active.id as GraphId, currentSplitTarget as GraphId));
      return;
    }

    if (currentDropGap !== null) {
      setLayout(moveGraphToIndex(layout, active.id as GraphId, currentDropGap));
      return;
    }
  };

  const sensors = useSensors(
    useSensor(PointerSensor, {
      activationConstraint: {
        distance: 0,
      },
    }),
  );

  const activeItem = activeId
    ? layout.find((item) => getItemId(item) === activeId)
    : null;

  return (
    <div ref={dropdownRef}>
      <div className={styles.dropdownButton} onClick={handleClick}>
        <RearrangeIcon />
        Rearrange Graphs
      </div>
      {isOpen && (
        <div className={styles.dropdownMenu}>
          <DndContext
            sensors={sensors}
            onDragStart={onDragStart}
            onDragEnd={onDragEnd}
          >
            <DropGap index={0} isActive={dropGapIndex === 0} />
            {layout.map((item, i) => {
              const id = getItemId(item);
              return (
                <Fragment key={id}>
                  <DraggableItem
                    id={id}
                    label={getItemLabel(item)}
                    isDragging={id === activeId}
                    isSplitTarget={id === splitTargetId}
                  />
                  <DropGap index={i + 1} isActive={dropGapIndex === i + 1} />
                </Fragment>
              );
            })}
            <DragOverlay>
              {activeItem ? (
                <div
                  className={styles.dropdownItem}
                  style={{ transform: 'scale(0.95)', opacity: 0.85, filter: 'brightness(0.75)', pointerEvents: 'none' }}
                >
                  <DragIndicatorIcon />
                  {getItemLabel(activeItem)}
                </div>
              ) : null}
            </DragOverlay>
          </DndContext>
        </div>
      )}
    </div>
  );
};

const mapStateToProps = (state: any) => ({
  graphs: state.graphs,
});

const mapDispatchToProps = {
  rearrangeGraphs: (newOrder: any) => {
    return {
      type: GraphsActions.REARRANGE_GRAPHS,
      payload: newOrder,
    };
  },
};

export default connect(
  mapStateToProps,
  mapDispatchToProps,
)(RearrangeGraphDropdown);
