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
  arrayMove,
  rectSortingStrategy,
} from '@dnd-kit/sortable';
import DragIndicatorIcon from '@/public/icons/drag_indicator.svg';
import styles from './stats.module.css';
import GraphsActions from '@/stores/Graphs/GraphsActions';
import { connect } from 'react-redux';

const graphsOrderNamesMap = {
  GPS: 'Speed',
  BatteriesVoltage: 'Batteries Voltage',
  BatteriesCurrent: 'Batteries Current',
  WindSensors: 'Wind Sensors',
};

const SortableItem = ({
  id,
  isDragging,
  isHalf,
}: {
  id: string;
  isDragging?: boolean;
  isHalf?: boolean;
}) => {
  const { attributes, listeners, setNodeRef, transform, transition } =
    useSortable({ id });

  const style = {
    transform: CSS.Transform.toString(transform),
    transition,
    opacity: isDragging ? 0.5 : 1,
    cursor: 'grab',
  };

  const itemClass = `${styles.dropdownItem} ${isHalf ? styles.dropdownItemHalf : styles.dropdownItemFull}`;

  return (
    <div
      className={itemClass}
      ref={setNodeRef}
      style={style}
      {...attributes}
      {...listeners}
    >
      <DragIndicatorIcon />
      {graphsOrderNamesMap[id as keyof typeof graphsOrderNamesMap]}
    </div>
  );
};

const RearrangeGraphDropdown = ({ graphs, rearrangeGraphs, setGraphLayout }: any) => {
  const [isOpen, setIsOpen] = useState(false);
  const [graphsOrder, setGraphsOrder] = useState(graphs.order);
  const [activeId, setActiveId] = useState<string | null>(null);
  const dropdownRef = useRef<HTMLDivElement>(null);

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
  
  const getIsRightDrop = (event: any) => { 
    const overRect = event.over?.rect; // graph to drop onto
    const activeRect = event.active?.rect?.current?.translated || event.active?.rect?.current; // graph to drag/drop
    
    if (!overRect || !activeRect) return null;
    
    const centerX = activeRect.left + activeRect.width / 2;
    const midX = overRect.left + overRect.width / 2;
    const threshold = Math.min(overRect.width * 0.25, 60); // deadzone
    
    if (Math.abs(centerX - midX) < threshold) return null;
    
    return centerX > midX;
  }

  const buildOrderWithSideInsert = (order: string[], sourceId: string, targetId: string, insertRight: boolean) => {
    // if dragging item onto itself no change needed
    if (sourceId === targetId) return order;
    
    // remove the source item from the list
    const base = order.filter((id) => id !== sourceId);
    
    // find where the target item is
    let idx = base.indexOf(targetId);
    if (idx === -1) return base;  // target not found, return as-is
    
    // if inserting to the right, move index forward by 1
    if (insertRight) idx += 1;
    
    // Insert source at the new position
    base.splice(idx, 0, sourceId);
    return base;
  };

  const applyLayoutPairAndNormalize = (order: string[], currentLayout: Record<string, 'full'|'half'>, pair: [string, string]) => {
    // create a copy of layout to modify
    const next = { ...currentLayout };
    
    // make both items in the pair half-width
    const [a, b] = pair;
    next[a] = 'half';
    next[b] = 'half';
    
    // loop through all graphs to find "orphan" half-width graphs
    for (let i = 0; i < order.length; i++) {
      const id = order[i];
      
      if (next[id] !== 'half') continue;
      
      // check neighbours
      const leftId = i > 0 ? order[i - 1] : null;
      const rightId = i < order.length - 1 ? order[i + 1] : null;
      
      // are neighbours half?
      const leftHalf = leftId && next[leftId] === 'half';
      const rightHalf = rightId && next[rightId] === 'half';
      
      // if no half-width neighbours, make it full-width
      if (!leftHalf && !rightHalf) next[id] = 'full';
    }
    
    return next;
  };
  useEffect(() => {
    rearrangeGraphs(graphsOrder);
  }, [graphsOrder]);

  const onDragOver = (event: any) => {
    const { active, over } = event;
    if (!over) {
      const containerRect = dropdownRef.current?.getBoundingClientRect();
      if (!containerRect) return;

      const activeRect = event.active.rect.current.translated;
      const oldIndex = graphsOrder.indexOf(active.id);
      let newIndex;

      if (activeRect.top < containerRect.top) {
        newIndex = 0;
      } else if (activeRect.bottom > containerRect.bottom) {
        newIndex = graphsOrder.length - 1;
      } else {
        return;
      }

      const newGraphsOrder = arrayMove(graphsOrder, oldIndex, newIndex);
      setGraphsOrder(newGraphsOrder);
    }
  };

  const onDragEnd = (event: any) => {
    const { active, over } = event;
    setActiveId(null);

    if (!over) return;
    if (active.id === over.id) return;

    const isRight = getIsRightDrop(event);

    if (isRight === null) {
      const oldIndex = graphsOrder.indexOf(active.id);
      const newIndex = graphsOrder.indexOf(over.id);
      setGraphsOrder(arrayMove(graphsOrder, oldIndex, newIndex));
      return;
    }

    // dropped on left or right -> place side-by-side
    const newOrder = buildOrderWithSideInsert(graphsOrder, active.id, over.id, isRight);
    setGraphsOrder(newOrder);

    const pair = isRight 
      ? [over.id, active.id] as [string, string]   // right drop: [target, dragged]
      : [active.id, over.id] as [string, string];  // left drop: [dragged, target] 
    
    const nextLayout = applyLayoutPairAndNormalize(newOrder, graphs.layout, pair);

    Object.keys(nextLayout).forEach((id) => { // update redux
      if (graphs.layout[id] !== nextLayout[id]) {
        setGraphLayout(id, nextLayout[id]);
      }
    });
  };


  const sensors = useSensors(
    useSensor(PointerSensor, {
      activationConstraint: {
        distance: 0,
      },
    }),
  );

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
          >
            <SortableContext
              items={graphsOrder}
              strategy={rectSortingStrategy}
            >
              {graphsOrder.map((id: any) => (
                <SortableItem
                  key={id}
                  id={id}
                  isDragging={id === activeId}
                  isHalf={graphs.layout[id] === 'half'}
                />
              ))}
            </SortableContext>
            <DragOverlay>
              {activeId ? (
                <div className={`${styles.dropdownItem} ${graphs.layout[activeId] === 'half' ? styles.dropdownItemHalf : styles.dropdownItemFull}`}>
                  <DragIndicatorIcon />
                  {
                    graphsOrderNamesMap[
                      activeId as keyof typeof graphsOrderNamesMap
                    ]
                  }
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
  setGraphLayout: (id: string, value: 'full' | 'half') => {
    return {
      type: GraphsActions.SET_GRAPH_LAYOUT,
      payload: { id, value },
    };
  }
};

export default connect(
  mapStateToProps,
  mapDispatchToProps,
)(RearrangeGraphDropdown);
