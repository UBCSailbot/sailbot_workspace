'use client';

import { CSS } from '@dnd-kit/utilities';
import { useSortable } from '@dnd-kit/sortable';
import { useState, useEffect, useRef } from 'react';
import RearrangeIcon from '@/public/icons/format_line_spacing.svg';
import {
  DndContext,
  DragOverlay,
  useSensor,
  useSensors,
  PointerSensor,
} from '@dnd-kit/core';
import {
  SortableContext,
  verticalListSortingStrategy,
} from '@dnd-kit/sortable';
import DragIndicatorIcon from '@/public/icons/drag_indicator.svg';
import styles from './stats.module.css';
import { restrictToVerticalAxis } from '@dnd-kit/modifiers';
import GraphsActions from '@/stores/Graphs/GraphsActions';
import { connect } from 'react-redux';
import { Layout, LayoutItem, GraphId, isSplitGroup } from '@/stores/Graphs/GraphsTypes';
import { moveGraph } from '@/stores/Graphs/GraphsLayoutHelpers';

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

const SortableItem = ({
  id,
  label,
  isDragging,
}: {
  id: string;
  label: string;
  isDragging?: boolean;
}) => {
  const { attributes, listeners, setNodeRef, transform, transition } =
    useSortable({ id });

  const style = {
    transform: CSS.Transform.toString(transform),
    transition,
    opacity: isDragging ? 0.5 : 1,
    cursor: 'grab',
  };

  return (
    <div
      className={styles.dropdownItem}
      ref={setNodeRef}
      style={style}
      {...attributes}
      {...listeners}
    >
      <DragIndicatorIcon />
      {label}
    </div>
  );
};

const RearrangeGraphDropdown = ({ graphs, rearrangeGraphs }: any) => {
  const [isOpen, setIsOpen] = useState(false);
  const [layout, setLayout] = useState<Layout>(graphs.layout);
  const [activeId, setActiveId] = useState<string | null>(null);
  const dropdownRef = useRef<HTMLDivElement>(null);

  const sortableIds = layout.map(getItemId);

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

  const onDragStart = (event: any) => {
    setActiveId(event.active.id);
  };

  useEffect(() => {
    rearrangeGraphs(layout);
  }, [layout]);

  const onDragOver = (event: any) => {
    const { active, over } = event;
    if (!over) {
      const containerRect = dropdownRef.current?.getBoundingClientRect();
      if (!containerRect) return;

      const activeRect = event.active.rect.current.translated;

      let targetId: GraphId;
      if (activeRect.top < containerRect.top) {
        targetId = getItemId(layout[0]) as GraphId;
      } else if (activeRect.bottom > containerRect.bottom) {
        targetId = getItemId(layout[layout.length - 1]) as GraphId;
      } else {
        return;
      }

      setLayout(moveGraph(layout, active.id as GraphId, targetId));
    }
  };

  const onDragEnd = (event: any) => {
    const { active, over } = event;
    setActiveId(null);

    if (!over) return;

    if (active.id !== over.id) {
      setLayout(moveGraph(layout, active.id as GraphId, over.id as GraphId));
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
            onDragOver={onDragOver}
            onDragEnd={onDragEnd}
            modifiers={[restrictToVerticalAxis]}
          >
            <SortableContext
              items={sortableIds}
              strategy={verticalListSortingStrategy}
            >
              {layout.map((item) => {
                const id = getItemId(item);
                return (
                  <SortableItem
                    key={id}
                    id={id}
                    label={getItemLabel(item)}
                    isDragging={id === activeId}
                  />
                );
              })}
            </SortableContext>
            <DragOverlay>
              {activeItem ? (
                <div className={styles.dropdownItem}>
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
