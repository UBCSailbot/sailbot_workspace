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
  verticalListSortingStrategy,
} from '@dnd-kit/sortable';
import styles from './stats.module.css';
import {
  restrictToVerticalAxis,
} from '@dnd-kit/modifiers';
import GraphsActions from '@/stores/Graphs/GraphsActions';
import { connect } from 'react-redux';

const graphsOrderNamesMap = {
  GPS: 'GPS',
  BatteriesVoltage: 'Batteries Voltage',
  BatteriesCurrent: 'Batteries Current',
  WindSensors: 'Wind Sensors',
};

const SortableItem = ({
  id,
  isDragging,
}: {
  id: string;
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
      {graphsOrderNamesMap[id as keyof typeof graphsOrderNamesMap]}
    </div>
  );
};

const RearrangeGraphDropdown = ({ graphs, rearrangeGraphs }: any) => {
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

    if (active.id !== over.id) {
      const oldIndex = graphsOrder.indexOf(active.id);
      const newIndex = graphsOrder.indexOf(over.id);
      const newGraphsOrder = arrayMove(graphsOrder, oldIndex, newIndex);

      setGraphsOrder(newGraphsOrder);
    }
  };

  const sensors = useSensors(
    useSensor(PointerSensor, {
      activationConstraint: {
        distance: 0,
      },
    }),
  );

  return (
    <div className={styles.dropdown} ref={dropdownRef}>
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
              items={graphsOrder}
              strategy={verticalListSortingStrategy}
            >
              {graphsOrder.map((id: any) => (
                <SortableItem key={id} id={id} isDragging={id === activeId} />
              ))}
            </SortableContext>
            <DragOverlay>
              {activeId ? (
                <div className={styles.dropdownItem}>
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
};

export default connect(
  mapStateToProps,
  mapDispatchToProps,
)(RearrangeGraphDropdown);
