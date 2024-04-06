export type Timestamps = {
  startDate: string | null;
  endDate: string | null;
};

export type DataFilterState = {
  timestamps: Timestamps;
  error?: any;
};
