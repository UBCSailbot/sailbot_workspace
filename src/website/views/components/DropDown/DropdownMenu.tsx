import React, { useState, useEffect } from 'react';
import styles from './dropdown.module.css';
import { connect } from 'react-redux';
import { DndContext, closestCenter } from '@dnd-kit/core';
import {
  SortableContext,
  verticalListSortingStrategy,
  useSortable,
  arrayMove,
} from '@dnd-kit/sortable';
import {
  restrictToVerticalAxis,
  restrictToWindowEdges,
} from '@dnd-kit/modifiers';
import { CSS } from '@dnd-kit/utilities';
import { saveSessionStorageData, loadSessionStorageData } from '@/utils/SessionStorage'
import GraphsActions from '@/stores/Graphs/GraphsActions'

const SortableGraph = ({ id, children, order, setOrder }) => {

  const { attributes, listeners, setNodeRef, transform, transition } = useSortable({
    id,
  })

  const style = {
    transition,
    transform: CSS.Transform.toString(transform),
  }

  return (
    <div
      ref={setNodeRef}
      style={style}
      {...attributes}
      {...listeners}
      className={styles.dropdownItem}
      suppressHydrationWarning={true}
    >
      {children}
    </div>
  );
}
interface DropDownMenuProps {
  rearrangeGraphs: () => void;
  graphsOrder: {
    order: string[];
  };
}

const DropdownMenu = ({ rearrangeGraphs, graphsOrder }) => {
  const [order, setOrder] = useState(graphsOrder.order);

  useEffect(() => {
    const currentOrder = loadSessionStorageData("Current Order");
    if (currentOrder) {
      setOrder(currentOrder);
    }
  }, [graphsOrder]);

  const onDragEnd = ({ active, over })  => {

    if (active.id === over.id){
      return;
    }

    const oldIndex = order.indexOf(active.id);
    const newIndex = order.indexOf(over.id);

    let newArray = arrayMove(order, oldIndex, newIndex);
    rearrangeGraphs(newArray);
    setOrder(newArray);
    saveSessionStorageData('Current Order', newArray);
  }

  return (
    <div className={styles.dropdownMenu}>
      <DndContext collisionDetection={closestCenter} onDragEnd={onDragEnd} modifiers={[restrictToVerticalAxis, restrictToWindowEdges]}>
        <SortableContext items={order} strategy={verticalListSortingStrategy}>
          {order.map((id) => (
            <SortableGraph key={id} id={id} order={order} setOrder={setOrder}>
              {id}
              <div className={styles.dragDropIndicator}>::</div>
            </SortableGraph>
          ))}
        </SortableContext>
      </DndContext>
    </div>
  )
}

const mapStateToProps = (state) => ({
  graphsOrder: state.graphs
});

const mapDispatchToProps = {
  rearrangeGraphs: (arrayArrangement: any) => {
    return {
      type: GraphsActions.REARRANGE_GRAPHS,
      payload: arrayArrangement
    }
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(DropdownMenu);
