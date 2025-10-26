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

After spinning up the database, it will keep running in the background until Docker is stopped.
You can just manually stop the database by running:

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

## Access dev server on a mobile device (optional)

Want to develop our website and see how it looks on your phone? Easy!
Note: This process will probably require a home internet connection.
UBC has some chastity "security" policies that will prevent one or more of these steps.

### 1. Run the website

On your computer, run any of the commands you would normally use suffixed with `:host`.
This is a variant of the command that will expose the website on all network hosts. This changes your server's attitude:
Before: "I will only accept connections from the same computer".
After: "I will accept any and all connections".
This is still safe, as long as you are using your own secure home network. Anything else won't be able to reach it at all.

_Example:_

```
➜  website git:(main) ✗ npm run web:dev:no-api:host

> web:dev:no-api:host
> cross-env DISABLE_API=true next dev -p 3005 -H 0.0.0.0

   ▲ Next.js 14.1.4
   - Local:        http://localhost:3005
   - Network:      http://0.0.0.0:3005
   - Environments: .env.development
```

You will notice the new line `- Network:      http://0.0.0.0:3005`. Yay!

### 2. Find the IP address of your computer

#### Windows

In powershell, type `ipconfig` and look for your `IPv4 Address`:

_Example:_

```
Wireless LAN adapter Wi-Fi:

   Connection-specific DNS Suffix  . :
   Link-local IPv6 Address . . . . . : fe80::1997:afee:6ac0:6a8%5
   IPv4 Address. . . . . . . . . . . : 192.168.1.164
   Subnet Mask . . . . . . . . . . . : 255.255.255.0
   Default Gateway . . . . . . . . . : 192.168.1.1
```

#### MacOS

In your Mac terminal, run: `ipconfig getifaddr en1`. Use `en0` if you use ethernet connection instead.

I don't know exactly what the output should look like, but with enough digging you should be able to find it.
TODO: update this instructions once I get my hands on someone that wisely uses Mac over Windows.

### 3. Connect (don't worry if this doesn't work)

We expose our website on port `3005`, so open your browser and enter `{your IP}:3005`.
If that doesn't work try `http://{your IP}:3005`.

_Example:_
`192.168.1.164:3005`

If this works, you should also be able to connect on your phone by going to the same URL!

### 4. Deal with stupid Windows

If you are using Windows and the above method worked for you, congratulations for the miracle.
Otherwise you will be sorely disappointed. We need to tell your firewall to allow this.
Open a powershell terminal as administrator and enter:
`New-NetFirewallRule -DisplayName "Next.js Dev Server" -Direction Inbound -LocalPort 3005 -Protocol TCP -Action Allow`

_Example:_

```
PS C:\WINDOWS\system32> New-NetFirewallRule -DisplayName "Next.js Dev Server" -Direction Inbound -LocalPort 3005 -Protocol TCP -Action Allow


Name                          : {d723d87f-07c1-4520-8acd-2b12d96f3536}
DisplayName                   : Next.js Dev Server
Description                   :
DisplayGroup                  :
Group                         :
Enabled                       : True
Profile                       : Any
Platform                      : {}
Direction                     : Inbound
Action                        : Allow
EdgeTraversalPolicy           : Block
LooseSourceMapping            : False
LocalOnlyMapping              : False
Owner                         :
PrimaryStatus                 : OK
Status                        : The rule was parsed successfully from the store. (65536)
EnforcementStatus             : NotApplicable
PolicyStoreSource             : PersistentStore
PolicyStoreSourceType         : Local
RemoteDynamicKeywordAddresses : {}
PolicyAppId                   :
PackageFamilyName             :
```

If you are not using WSL try it again!
Otherwise, you will need to forward port 3005 from your network to WSL.
In your WSL terminal, run: `ip route` to get your ip info:

_Example:_

```
➜  sailbot_workspace git:(main) ✗ ip route
default via 172.26.224.1 dev eth0 proto kernel
172.26.224.0/20 dev eth0 proto kernel scope link src 172.26.237.182
```

In your administrator powershell, type:
`netsh interface portproxy add v4tov4 listenport=3005 listenaddress=0.0.0.0 connectport=3005 connectaddress={your WSL IP}`:

_Example:_

```
PS C:\WINDOWS\system32> netsh interface portproxy add v4tov4 listenport=3005 listenaddress=0.0.0.0 connectport=3005 connectaddress=172.26.237.182
```

### I ran into an error in this process. What can I do?

`https://chatgpt.com/`
