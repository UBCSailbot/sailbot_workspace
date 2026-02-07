export type GraphId = 'GPS' | 'BatteriesVoltage' | 'BatteriesCurrent' | 'WindSensors';

// A layout item is either a single graph or a split group of graphs
export type LayoutItem = GraphId | GraphId[];

// The full layout is an array of layout items displayed sequentially
export type Layout = LayoutItem[];

export type GraphsState = {
  layout: Layout;
  error?: any;
};

// Helper to check if a layout item is a split group
export const isSplitGroup = (item: LayoutItem): item is GraphId[] => {
  return Array.isArray(item);
};

// Helper to get all graph IDs from the layout (flattened)
export const getAllGraphIds = (layout: Layout): GraphId[] => {
  return layout.flatMap((item) => (isSplitGroup(item) ? item : [item]));
};
