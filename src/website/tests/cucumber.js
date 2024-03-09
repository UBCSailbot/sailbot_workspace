module.exports = {
  default: [
    '--require-module ts-node/register',
    '--require features/**/*.ts',
    '--require steps/**/*.ts', // Load step definitions
    '--require shared/**/*.ts', // Shared utility functions
    '--require world/**/*.ts', // Shared utility functions
  ].join(' '),
};
