export type GraphId = 'GPS' | 'BatteriesVoltage' | 'BatteriesCurrent' | 'WindSensors';

export type GraphsState = {
  order: GraphId[];
  layout: Record<GraphId, 'full' | 'half'>;
  error?: any;
};
