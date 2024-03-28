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

const SortableGraph = ({ id, children, order, dispatch, setOrder }) => {

  const { attributes, listeners, setNodeRef, transform, transition } = useSortable({
    id,
  })

  const style = {
    transition,
    transform: CSS.Transform.toString(transform),
  }

  // const handleRightClick = (event) => {
  //   event.preventDefault();

  //   function removeGraph(graphInOrder) {
  //     console.log(id + " was clicked")
  //     return (id !== graphInOrder)
  //   }

  //   let filteredOrder = order.filter(removeGraph)
  //   console.log(filteredOrder)
  //   dispatch({type: 'REARRANGE_GRAPHS', payload: filteredOrder})
  //   setOrder(filteredOrder)
  // }

  return (
    <div
      ref={setNodeRef}
      style={style}
      {...attributes}
      {...listeners}
      className={styles.dropdownItem}
      // onContextMenu={handleRightClick}
      suppressHydrationWarning={true} // I know this is irresponsible but the warning is very annoying and i have no idea how to fix it ill ask one of you later
    >
      {children}
    </div>
  );
}

const DropdownMenu = ({ dispatch, graphsOrder }) => {
  const [order, setOrder] = useState(graphsOrder.order);

  useEffect(() => {
    const currentOrder = sessionStorage.getItem('Current Order')
    const storedOrder = JSON.parse(currentOrder);
    if (storedOrder) {
      setOrder(storedOrder);
    }
  }, [graphsOrder]);

  const onDragEnd = ({ active, over })  => {

    if (active.id === over.id){
      return;
    }

    const oldIndex = order.indexOf(active.id);
    const newIndex = order.indexOf(over.id);

    let newArray = arrayMove(order, oldIndex, newIndex);
    dispatch({type: 'REARRANGE_GRAPHS', payload: newArray});
    setOrder(newArray);
    sessionStorage.setItem('Current Order', JSON.stringify(newArray));
  }

  return (
    <div className={styles.dropdownMenu}>
      <DndContext collisionDetection={closestCenter} onDragEnd={onDragEnd} modifiers={[restrictToVerticalAxis, restrictToWindowEdges]}>
        <SortableContext items={order} strategy={verticalListSortingStrategy}>
          {order.map((id) => (
            <SortableGraph key={id} id={id} order={order} dispatch={dispatch} setOrder={setOrder}>
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

export default connect(mapStateToProps)(DropdownMenu);
