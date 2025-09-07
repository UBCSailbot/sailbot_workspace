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
import { restrictToVerticalAxis } from '@dnd-kit/modifiers';

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
    cursor: 'grab',
    opacity: isDragging ? 0.5 : 1,
  };

  return (
    <div
      className={styles.dropdownItem}
      ref={setNodeRef}
      style={style}
      {...attributes}
      {...listeners}
    >
      Item {id}
    </div>
  );
};

const RearrangeGraphDropdown = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [items, setItems] = useState(['1', '2', '3', '4']);
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

  const onDragEnd = (event: any) => {
    const { active, over } = event;
    setActiveId(null);

    if (!over) return;

    if (active.id !== over.id) {
      setItems((items) => {
        const oldIndex = items.indexOf(active.id);
        const newIndex = items.indexOf(over.id);
        return arrayMove(items, oldIndex, newIndex);
      });
    }
  };

  const sensors = useSensors(
    useSensor(PointerSensor, {
      // Activate immediately on pointer down
      activationConstraint: {
        distance: 0,
      },
    }),
  );

  const onDragMove = (event: any) => {
    if (!dropdownRef.current) return;

    const { active, delta } = event;
    const containerRect = dropdownRef.current.getBoundingClientRect();
    const activeRect = event.active.rect.current.translated;

    // Check if dragged item is above or below container
    if (activeRect.top < containerRect.top) {
      // Move to top
      setItems((items) => {
        const oldIndex = items.indexOf(active.id);
        if (oldIndex === 0) return items;
        return arrayMove(items, oldIndex, 0);
      });
    } else if (activeRect.bottom > containerRect.bottom) {
      // Move to bottom
      setItems((items) => {
        const oldIndex = items.indexOf(active.id);
        if (oldIndex === items.length - 1) return items;
        return arrayMove(items, oldIndex, items.length - 1);
      });
    }
  };

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
            onDragMove={onDragMove}
            onDragEnd={onDragEnd}
            modifiers={[restrictToVerticalAxis]}
          >
            <SortableContext
              items={items}
              strategy={verticalListSortingStrategy}
            >
              {items.map((id) => (
                <SortableItem key={id} id={id} isDragging={id === activeId} />
              ))}
            </SortableContext>
            <DragOverlay>
              {activeId ? (
                <div className={styles.dropdownItem}>Item {activeId}</div>
              ) : null}
            </DragOverlay>
          </DndContext>
        </div>
      )}
    </div>
  );
};

export default RearrangeGraphDropdown;
