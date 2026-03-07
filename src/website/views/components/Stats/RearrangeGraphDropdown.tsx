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
import { Layout, GraphId, isSplitGroup } from '@/stores/Graphs/GraphsTypes';
import { extractGraph, splitGraph, findLayoutIndex } from '@/stores/Graphs/GraphsLayoutHelpers';

const graphsOrderNamesMap: Record<GraphId, string> = {
  GPS: 'Speed',
  BatteriesVoltage: 'Batteries Voltage',
  BatteriesCurrent: 'Batteries Current',
  WindSensors: 'Wind Sensors',
};

type SplitSide = 'left' | 'right' | 'full' | null;

const DraggableItem = ({
  id,
  label,
  isDragging,
  splitTargetSide,
  layoutIndex,
}: {
  id: string;
  label: string;
  isDragging?: boolean;
  splitTargetSide?: SplitSide;
  layoutIndex?: number;
}) => {
  const { attributes, listeners, setNodeRef } = useDraggable({ id });

  const style = {
    opacity: isDragging ? 0.35 : 1,
    cursor: 'grab',
  };

  const splitClass =
    splitTargetSide === 'full' ? styles.dropdownItemSplitTarget :
    splitTargetSide === 'left' ? styles.dropdownItemSplitTargetLeft :
    splitTargetSide === 'right' ? styles.dropdownItemSplitTargetRight :
    '';
  const className = splitClass ? `${styles.dropdownItem} ${splitClass}` : styles.dropdownItem;

  return (
    <div
      className={className}
      ref={setNodeRef}
      style={style}
      data-sortable-id={id}
      data-layout-index={layoutIndex}
      {...attributes}
      {...listeners}
    >
      <DragIndicatorIcon />
      <span className={styles.dropdownItemLabel}>{label}</span>
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
  const [splitSide, setSplitSide] = useState<SplitSide>(null);
  const [dropGapIndex, setDropGapIndex] = useState<number | null>(null);
  const dropdownRef = useRef<HTMLDivElement>(null);
  const hoverTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const hoverTargetRef = useRef<string | null>(null);
  const splitTargetRef = useRef<string | null>(null);
  const splitSideRef = useRef<SplitSide>(null);
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

  const updateSplitSide = (side: SplitSide) => {
    splitSideRef.current = side;
    setSplitSide(side);
  };

  const updateDropGap = (index: number | null) => {
    dropGapRef.current = index;
    setDropGapIndex(index);
  };

  const onDragStart = (event: any) => {
    clearHoverTimer();
    updateSplitTarget(null);
    updateSplitSide(null);
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
    const sourceItem = layout[sourceIndex];
    const activeIsInSplitGroup = isSplitGroup(sourceItem);

    const handlePointerMove = (e: PointerEvent) => {
      // Check items via elementsFromPoint (merge/split intent)
      const elements = document.elementsFromPoint(e.clientX, e.clientY);
      for (const el of elements) {
        const sortableId = (el as HTMLElement).dataset?.sortableId;
        if (sortableId && sortableId !== activeId) {
          // If both graphs are in the same split group, skip — fall through to gap detection
          const targetLayoutIndex = findLayoutIndex(layout, sortableId as GraphId);
          if (sourceIndex === targetLayoutIndex) break;

          updateDropGap(null);

          // Compute merge side in real-time on every pointer move
          const targetItem = layout[targetLayoutIndex];
          const isFullGroup = isSplitGroup(targetItem) && targetItem.length >= 2;
          if (isFullGroup) {
            updateSplitSide('full');
          } else {
            const rect = (el as HTMLElement).getBoundingClientRect();
            updateSplitSide(e.clientX < rect.left + rect.width / 2 ? 'left' : 'right');
          }

          if (sortableId !== hoverTargetRef.current) {
            clearHoverTimer();
            updateSplitTarget(null);
            hoverTargetRef.current = sortableId;
            hoverTimerRef.current = setTimeout(() => {
              updateSplitTarget(sortableId);
            }, 100);
          }
          return;
        }
      }

      // Determine gap from Y position using layout-level elements
      const itemEls = Array.from(document.querySelectorAll<HTMLElement>('[data-layout-index]'));
      if (itemEls.length === 0) return;

      const itemRects = itemEls.map((el) => el.getBoundingClientRect());

      let gapIndex = itemRects.length;
      for (let i = 0; i < itemRects.length; i++) {
        const midY = (itemRects[i].top + itemRects[i].bottom) / 2;
        if (e.clientY < midY) {
          gapIndex = i;
          break;
        }
      }

      // Skip gaps adjacent to the dragged item only for standalone items.
      // For items in a split group, adjacent gaps are meaningful (extraction).
      if (!activeIsInSplitGroup && (gapIndex === sourceIndex || gapIndex === sourceIndex + 1)) {
        clearHoverTimer();
        updateSplitTarget(null);
        updateSplitSide(null);
        updateDropGap(null);
        return;
      }

      clearHoverTimer();
      updateSplitTarget(null);
      updateSplitSide(null);
      updateDropGap(gapIndex);
    };

    document.addEventListener('pointermove', handlePointerMove);
    return () => {
      document.removeEventListener('pointermove', handlePointerMove);
      clearHoverTimer();
      updateSplitTarget(null);
      updateSplitSide(null);
      updateDropGap(null);
    };
  }, [activeId, layout]);

  const onDragEnd = (event: any) => {
    const { active } = event;
    const currentSplitTarget = splitTargetRef.current;
    const currentSplitSide = splitSideRef.current;
    const currentDropGap = dropGapRef.current;

    clearHoverTimer();
    updateSplitTarget(null);
    updateSplitSide(null);
    updateDropGap(null);
    setActiveId(null);

    if (currentSplitTarget) {
      const side = currentSplitSide === 'left' ? 'left' : 'right';
      setLayout(splitGraph(layout, active.id as GraphId, currentSplitTarget as GraphId, side));
      return;
    }

    if (currentDropGap !== null) {
      // extractGraph handles both standalone (delegates to moveGraphToIndex)
      // and split group members (pulls out and places at gap)
      setLayout(extractGraph(layout, active.id as GraphId, currentDropGap));
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

  const activeLabel = activeId ? graphsOrderNamesMap[activeId as GraphId] : null;

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
            {layout.map((item, i) => (
              <Fragment key={isSplitGroup(item) ? item.join('-') : item}>
                {isSplitGroup(item) ? (
                  <div className={styles.splitGroupDropdown} data-layout-index={i}>
                    {item.map((graphId) => (
                      <DraggableItem
                        key={graphId}
                        id={graphId}
                        label={graphsOrderNamesMap[graphId]}
                        isDragging={graphId === activeId}
                        splitTargetSide={graphId === splitTargetId ? 'full' : null}
                      />
                    ))}
                  </div>
                ) : (
                  <DraggableItem
                    id={item}
                    label={graphsOrderNamesMap[item]}
                    isDragging={item === activeId}
                    splitTargetSide={item === splitTargetId ? splitSide : null}
                    layoutIndex={i}
                  />
                )}
                <DropGap index={i + 1} isActive={dropGapIndex === i + 1} />
              </Fragment>
            ))}
            <DragOverlay>
              {activeLabel ? (
                <div
                  className={styles.dropdownItem}
                  style={{ transform: 'scale(0.90)', opacity: 0.85, filter: 'brightness(0.75)', pointerEvents: 'none' }}
                >
                  <DragIndicatorIcon />
                  <span className={styles.dropdownItemLabel}>{activeLabel}</span>
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
