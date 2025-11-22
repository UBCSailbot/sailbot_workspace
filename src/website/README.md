# Website

In the website development timeline, we are currently evaluating the folllowing software stack:
[Next.js](https://nextjs.org/) website (this repository), [Typescript](https://www.typescriptlang.org/docs/),
[React](https://react.dev/) + [Redux](https://redux-saga.js.org/), and the [MongoDB](https://www.mongodb.com/) database.
The easiest way to evaluate these potential solutions for our purposes is in [sailbot_workspace](https://github.com/UBCSailbot/sailbot_workspace).

## Database

[MongoDB](https://www.mongodb.com/) is a general purpose, document-based, distributed database built for modern application
developers and for the cloud era. If you want to learn more about MongoDB, visit their docs site: [MongoDB Documentation](https://docs.mongodb.com/).

## Setup

### Environment variables

We have two separate configurations: one for development `.env.development`,
the other for production `.env.production`. The values may vary, but the environment variables are the same. See below:

- `MONGODB_URI`: Your MongoDB connection string. Use `mongodb://localhost:27017/<DB_NAME>` to establish a connection
  with the local database.
- `NEXT_PUBLIC_SERVER_HOST`: The host URL of the website.
- `NEXT_PUBLIC_SERVER_PORT`: The port number of the website.
- `NEXT_PUBLIC_POLLING_TIME_MS`: The time interval for polling the database in milliseconds.
- `AISSTREAM_API_KEY`: API key from aisstream.io (kept server-side only).
- `AISSTREAM_BOUNDING_BOXES`: JSON string of bounding boxes (e.g. [[[-90,-180],[90,180]]]); set this to your sailing region to limit traffic.
- `AISSTREAM_FILTER_SHIP_MMSI`: Optional comma-separated MMSI allowlist.
- `AISSTREAM_FILTER_MESSAGE_TYPES`: Optional comma-separated AIS message types. Defaults to Position and Static types.
- `AISSTREAM_FLUSH_INTERVAL_MS`: How often to write snapshots to MongoDB (default 15000 ms).
- `AISSTREAM_MAX_SNAPSHOTS`: How many snapshots to retain before trimming (default 50).

### Package installation

The following command installs all required dependencies listed in the `package.json` file:

```
npm install
```

Once the installation is complete, you should see a `node_modules` directory in the project's root.
This directory contains all installed packages.

When installing a new package to the website, please follow the steps below:

1. Run the command `npm install <package-name>`.
   Replace `<package-name>` with the actual name of the package you want to add.

   - Should you encounter errors related to resolving peer dependencies, please re-run the command with
     the header `--legacy-peer-deps`. Do not to use `--force` unless you're well aware of the potential consequences.

2. Review the `package.json` file to ensure the new package and its version have been added to the dependencies section.
   - Confirm that `package-lock.json` has also been updated.
     This file holds specific version information to ensure consistent installations across different environments.
3. Once the installation process is finished, please make sure to commit the files `package.json` and `package-lock.json`.
   These files are essential for version controlling the dependencies that have been added.

## Run

You can run the website in development mode by executing the following command:

```bash
npm run dev
```

This will start the website in development mode and spin up a Docker container for our MongoDB database.
Make sure you have Docker installed!

Once you have run the website in development mode, you can access it at [http://localhost:3005](http://localhost:3005).

After spinning up the database, it will keep running in the background until Docker is stopped. You can just manually stop the database by running:

```bash
npm run db:stop
```

Otherwise, here are some useful commands:

| Command              | Description                                              |
| -------------------- | -------------------------------------------------------- |
| `npm run db:start`   | Starts the MongoDB database container                    |
| `npm run db:stop`    | Stops the MongoDB database container                     |
| `npm run web:dev`    | Runs the website in development mode on port 3005        |
| `npm run dev`        | Starts both the database and website in development mode |
| `npm run web:build`  | Builds the website for production                        |
| `npm run web:start`  | Starts the production website on port 3005               |
| `npm run simulation` | Runs the simulation script                               |
| `node scripts/aisstream-ingest.js` | Starts the AIS ingest worker that pulls data from aisstream.io into MongoDB |

## Linters

Before merging in new changes to the repository, please execute the following commands in order:

```bash
npm run format
```

This command runs [Prettier](https://prettier.io/docs/en/index.html) to automatically format the code according to
the rules defined in the configuration file `.prettierrc`.

```bash
npm run lint
```

This command runs [ESLint](https://eslint.org/docs/latest/use/getting-started) to analyze the code for potential errors
and enforce coding style based on the rules defined in the configuration file `.eslintrc`.
