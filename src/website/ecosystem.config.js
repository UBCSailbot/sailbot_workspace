module.exports = {
  apps: [
    {
      name: 'sailbot-website',
      script: 'server.js',
      cwd: '/root/sailbot_workspace/src/website',
      instances: 1,
      exec_mode: 'cluster',
      autorestart: true,
      watch: false,
      max_memory_restart: '1G',
      env: {
        NODE_ENV: 'production',
        PORT: 3005,
        MONGODB_URI: process.env.MONGODB_URI || 'mongodb://localhost:27017/sailbot',
        NEXT_PUBLIC_SERVER_HOST: process.env.NEXT_PUBLIC_SERVER_HOST || 'http://localhost',
        NEXT_PUBLIC_SERVER_PORT: process.env.NEXT_PUBLIC_SERVER_PORT || '3005',
        NEXT_PUBLIC_POLLING_TIME_MS: process.env.NEXT_PUBLIC_POLLING_TIME_MS || '1000',
      },
      error_file: '/root/sailbot_workspace/logs/website-error.log',
      out_file: '/root/sailbot_workspace/logs/website-out.log',
      log_date_format: 'YYYY-MM-DD HH:mm:ss Z',
      merge_logs: true,
    },
  ],
};

