export type Timestamp = {
    startDate: String | null,
    endDate: String | null
}

export type TimestampState = {
    timestamps: Timestamp[];
    error?: any;
}
