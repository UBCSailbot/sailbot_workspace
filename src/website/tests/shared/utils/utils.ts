import winston from 'winston';

const logFormat = winston.format.printf(function (log) {
  return JSON.stringify(log.message, null, ' ');
});

export const logger = winston.createLogger({
  transports: [
    new winston.transports.Console({
      level: 'error',
      format: logFormat,
    }),
  ],
});

export function convertBigIntToString(input: bigint): any {
  if (typeof input == 'bigint') {
    return input.toString();
  }
  return input;
}
