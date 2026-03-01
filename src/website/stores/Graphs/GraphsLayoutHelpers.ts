import { Layout, LayoutItem, GraphId, isSplitGroup } from './GraphsTypes';

/**
 * Find the index of the layout item containing a given graphId.
 * Returns -1 if not found.
 */
export const findLayoutIndex = (layout: Layout, graphId: GraphId): number => {
  return layout.findIndex((item) =>
    isSplitGroup(item) ? item.includes(graphId) : item === graphId,
  );
};

/**
 * Remove a graphId from the layout.
 * If the graph was in a split group, the group is preserved (or collapsed to
 * a standalone item when only one graph remains).
 */
const removeGraph = (layout: Layout, graphId: GraphId): Layout => {
  return layout
    .map((item): LayoutItem | null => {
      if (isSplitGroup(item)) {
        const filtered = item.filter((id) => id !== graphId);
        if (filtered.length === 0) return null;
        if (filtered.length === 1) return filtered[0];
        return filtered;
      }
      return item === graphId ? null : item;
    })
    .filter((item): item is LayoutItem => item !== null);
};

/**
 * Move the layout item containing sourceId to the position of the item
 * containing targetId. Split groups are treated as atomic units.
 *
 * Returns the original layout unchanged if:
 * - sourceId or targetId is not found
 * - both IDs belong to the same layout item
 */
export const moveGraph = (
  layout: Layout,
  sourceId: GraphId,
  targetId: GraphId,
): Layout => {
  if (sourceId === targetId) return layout;

  const sourceIndex = findLayoutIndex(layout, sourceId);
  const targetIndex = findLayoutIndex(layout, targetId);

  if (sourceIndex === -1 || targetIndex === -1) return layout;
  if (sourceIndex === targetIndex) return layout;

  const newLayout = [...layout];
  const [removed] = newLayout.splice(sourceIndex, 1);

  const adjustedTargetIndex =
    sourceIndex < targetIndex ? targetIndex - 1 : targetIndex;
  newLayout.splice(adjustedTargetIndex, 0, removed);

  return newLayout;
};

/**
 * Create a vertical split by moving sourceId into the same group as targetId.
 * If targetId is already in a split group, sourceId is appended to it.
 * If targetId is standalone, a new split group [targetId, sourceId] is created.
 *
 * Enforces:
 * - Vertical-only (no nested split groups)
 * - No duplicate graph IDs
 *
 * Returns the original layout unchanged if:
 * - sourceId or targetId is not found
 * - both IDs already belong to the same layout item
 */
/**
 * Move the layout item containing sourceId to a specific index position.
 * Gap index semantics: gap i = "insert before item[i]", gap N = "insert at end".
 * Gaps adjacent to the source item (sourceIndex and sourceIndex + 1) are no-ops.
 */
export const moveGraphToIndex = (
  layout: Layout,
  sourceId: GraphId,
  targetIndex: number,
): Layout => {
  const sourceIndex = findLayoutIndex(layout, sourceId);
  if (sourceIndex === -1) return layout;
  if (targetIndex === sourceIndex || targetIndex === sourceIndex + 1) return layout;

  const newLayout = [...layout];
  const [removed] = newLayout.splice(sourceIndex, 1);
  const adjustedIndex = targetIndex > sourceIndex ? targetIndex - 1 : targetIndex;
  newLayout.splice(adjustedIndex, 0, removed);
  return newLayout;
};

/**
 * Extract a single graph out of its split group and insert it as a standalone
 * item at targetIndex (gap semantics: gap i = "insert before item[i]").
 * If the graph is already standalone, delegates to moveGraphToIndex.
 */
export const extractGraph = (
  layout: Layout,
  graphId: GraphId,
  targetIndex: number,
): Layout => {
  const sourceIndex = findLayoutIndex(layout, graphId);
  if (sourceIndex === -1) return layout;

  const sourceItem = layout[sourceIndex];
  if (!isSplitGroup(sourceItem)) {
    return moveGraphToIndex(layout, graphId, targetIndex);
  }

  // removeGraph collapses the group but never removes it (≥2 members → ≥1 remain),
  // so the layout item count stays the same and gap indices remain valid.
  const withoutGraph = removeGraph(layout, graphId);
  const newLayout = [...withoutGraph];
  newLayout.splice(targetIndex, 0, graphId);
  return newLayout;
};

export const splitGraph = (
  layout: Layout,
  sourceId: GraphId,
  targetId: GraphId,
): Layout => {
  if (sourceId === targetId) return layout;

  const sourceIndex = findLayoutIndex(layout, sourceId);
  const targetIndex = findLayoutIndex(layout, targetId);

  if (sourceIndex === -1 || targetIndex === -1) return layout;
  if (sourceIndex === targetIndex) return layout;

  const withoutSource = removeGraph(layout, sourceId);

  return withoutSource.map((item) => {
    if (isSplitGroup(item) && item.includes(targetId)) {
      return [...item, sourceId];
    }
    if (item === targetId) {
      return [targetId, sourceId];
    }
    return item;
  });
};
