export type Timestamp = {
    startDate: string | null,
    endDate: string | null
}

export type TimestampState = {
    timestamps: Timestamp[];
    error?: any;
}
