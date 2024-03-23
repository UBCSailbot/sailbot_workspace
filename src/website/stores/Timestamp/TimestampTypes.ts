export type Timestamp = {
  startDate: string | null;
  endDate: string | null;
};

export type TimestampState = {
  timestamp: Timestamp;
  error?: any;
};
