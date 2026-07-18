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
