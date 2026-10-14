<!-- markdownlint-disable MD013 -->

# MongoDB basics for Postgres + Drizzle developers

A baseline for understanding the database behind the POLARIS website. It compares everything to Postgres + Drizzle ORM and uses real examples from this repo. Mongoose, the library the website uses, plays roughly the role Drizzle plays in a SvelteKit app.

## 1. The mental model

MongoDB is a **document database**. Instead of rows with fixed columns, it stores **JSON-like objects** that can contain nested objects and arrays.

| Postgres                      | MongoDB                 | Notes                                                           |
| ----------------------------- | ----------------------- | --------------------------------------------------------------- |
| Database                      | Database                | Same idea. Dev DB here: `sailbot_db` (`.env.development`)       |
| Schema (namespace)            | (none)                  | Mongo has no namespace level between database and collection    |
| Table                         | **Collection**          | e.g. `gps`, `ais_ships`, `batteries`                            |
| Row                           | **Document**            | One JSON-like object                                            |
| Column                        | **Field**               | But each document can have different fields                     |
| Primary key (`serial`/`uuid`) | `_id`                   | Added automatically if you don't supply one (an `ObjectId`)     |
| Foreign key                   | (none)                  | You can store another document's `_id`, but nothing enforces it |
| `JOIN`                        | `$lookup` (rarely used) | Data is usually embedded instead of joined (see §5)             |
| `CREATE TABLE` / migrations   | (none)                  | A collection is created the first time you insert into it       |
| SQL                           | Query objects           | `{ speed: { $gt: 5 } }` instead of `WHERE speed > 5`            |

## 2. Documents and BSON

Documents are stored as **BSON** (Binary JSON), which is JSON plus extra types: `ObjectId`, `Date`, 32/64-bit integers, `double`, `Decimal128`, binary data, and so on.

A real GPS document, as written by the boat's remote server:

```js
{
  _id: ObjectId("66c4a1f2e4b0a1b2c3d4e5f6"),
  latitude: 48.834475,     // double
  longitude: -125.137001,  // double
  speed: 2.1,
  heading: 270.0,
  timestamp: "26-08-20 14:03:22"  // a string, not a Date
}
```

A batteries document shows the **embedded array** pattern (this would be a child table in Postgres):

```js
{
  _id: ObjectId("..."),
  batteries: [ { voltage: 12.4, current: 1.1 }, { voltage: 12.3, current: 0.9 } ],
  timestamp: "..."
}
```

An `ais_ships` document is a whole **snapshot** in one document: an array of every nearby ship at one moment.

## 3. Schema: the biggest difference

**Postgres enforces the schema.** You define tables in Drizzle (`pgTable(...)`), generate a migration with drizzle-kit, and the database rejects any row that doesn't fit.

**MongoDB enforces nothing by default.** Any document can go into any collection: different fields, different types, missing fields. (MongoDB can enforce a `$jsonSchema` validator on a collection, but this repo doesn't use one.)

So where does the schema live? **In application code, through Mongoose.** A Mongoose schema is like a Drizzle table definition, except:

- It's checked **only when your app writes through that Mongoose model**. The database itself knows nothing about it.
- There are **no migrations.** Changing a Mongoose schema doesn't touch existing documents. Old documents keep their old shape forever unless you write a script to update them.

### Why this matters in this repo

The website app (`app/api/*`) only **reads** from MongoDB: every route is a `GET` and calls only `find`/`findOne`. All the writes come from other programs, and **none of them use the website's Mongoose models** (`models/*.ts`). The one exception is the Cucumber API tests (`tests/steps/given.ts`), which clear the DB and seed it through those models:

| Writer                                                  | Language         | Collections                                                                                                                                        | Schema it follows                    |
| ------------------------------------------------------- | ---------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------ |
| `src/network_systems/projects/remote_server_standalone` | C++ (`mongocxx`) | `gps`, `ais_ships`, `batteries`, `wind_sensors`, `temp_sensors`, `ph_sensors`, `salinity_sensors`, `local_path`, `global_path`, `iridium_response` | Whatever `src/sailbot_db.cpp` builds |
| `src/website/scripts/aisstream-ingest.js`               | Node (Mongoose)  | `aisships` (see mismatch below)                                                                                                                    | Its own copy of the AISShips schema  |
| `src/website/tests/simulation/simulation.py`            | Python           | simulated versions of the above                                                                                                                    | Its own code + JSON fixtures         |

So the Mongoose schemas in `src/website/models/` describe what the website **expects**, not what is guaranteed to be there. Some mismatches I found (verified against the code, 2026-10-11):

- **Collection name mismatch for global path.** `models/GlobalPath.ts` doesn't set a collection name, so Mongoose derives one from the model name: `GlobalPath` → `globalpaths`. The C++ server writes to `global_path` (`inc/sailbot_db.h`). The simulation script writes to `globalpaths`, so it works in dev, but real boat data would never reach the website. (The global path display is currently hidden in the UI.)
- **Collection name mismatch for AIS ships.** `scripts/aisstream-ingest.js` defines its own Mongoose model `AISShips` (line ~206) without a collection name, so it writes to `aisships`. Commit `4f90ccd3` (Aug 15 2026) changed the website's `models/AISShips.ts` to read `ais_ships` instead (matching the C++ server). Since then, the website reads the C++ server's AIS data and **not** the aisstream worker's, unless production overrides this somehow. Unverified against the production DB.
- **Number types.** The models declare coordinates as `Decimal128`, but the C++ server writes plain `double`s. This works because the API routes use `.lean()` (which skips Mongoose's type conversion) and `models/helper/parser.js` only converts values that really are `Decimal128`.
- **Timestamp formats.** Mongoose's default timestamp is ISO 8601 (`2026-08-20T14:03:22.000Z`). The C++ server writes `YY-MM-DD HH:MM:SS` (two-digit year, no timezone), which is RockBLOCK's `transmit_time` passed through unchanged. Both are strings, not BSON `Date`s, so sorting and filtering by time relies on string comparison, which only works when every document uses the same format.

### How the website actually uses its Mongoose schemas

In Mongoose you can't query a collection directly. You query a **Model**, and a model is built from a schema plus a collection name:

```ts
// models/GPS.ts
const GPSSchema = new mongoose.Schema({ latitude: ..., timestamp: ... });
export default mongoose.model('GPS', GPSSchema); // 3rd argument (collection name) omitted → Mongoose picks one

// app/api/gps/route.ts
GPS.find({}).select('-_id -__v').sort({ _id: -1 }).limit(n).lean();
```

So every query goes through a schema, but what the schema actually _does_ depends on the stage:

| Stage                         | What the schema does                                                                                     | Used in this repo?                                                          |
| ----------------------------- | -------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------- |
| Choosing the collection       | Maps the model to a collection name                                                                      | **Always.** This is where the name mismatches come from                     |
| Filter                        | Converts filter values to schema types (`{ speed: "5" }` → `5`)                                          | No. Every route filters with `{}`                                           |
| Results **without** `.lean()` | Wraps results in Mongoose documents with schema types; `res.json()` runs the schema's `toJSON` transform | Only `globalpath` and `localpath`                                           |
| Results **with** `.lean()`    | Nothing. Raw objects, exactly as the writer stored them                                                  | The other 8 routes                                                          |
| Writes                        | Validation (`required`, the "exactly 2 batteries" check), defaults, dropping unknown fields              | **Never in production.** Only the Cucumber tests write through these models |

`.lean()` isn't a mistake. It's the recommended approach for read-only endpoints, because it's faster and uses less memory. And Mongoose isn't strictly required: `.lean()` skips the schema on results, and `mongoose.connection.db.collection('gps').find({})` skips Mongoose entirely and uses the native driver.

**In practice the schemas shape no data a visitor sees.** The two routes that apply them are for hidden features, and `globalpath` reads the wrong collection anyway. **The website's Mongoose schemas define the collection names and give developers TypeScript types, but they don't validate or shape the data.** The real schema is set independently by each writer (the C++ server, the AIS worker, the simulation script), and nothing checks that they agree.

## 4. Querying: SQL, Drizzle, and Mongo side by side

"Get the 100 most recent GPS fixes, without `_id`":

```sql
-- SQL
SELECT latitude, longitude, speed, heading, timestamp
FROM gps ORDER BY id DESC LIMIT 100;
```

```ts
// Drizzle
db.select().from(gps).orderBy(desc(gps.id)).limit(100);
```

```js
// MongoDB shell (mongosh)
db.gps.find({}, { _id: 0 }).sort({ _id: -1 }).limit(100);
//          ^filter ^projection (which fields to return)
```

```ts
// Mongoose, as in app/api/gps/route.ts
GPS.find({}).select('-_id -__v').sort({ _id: -1 }).limit(100).lean();
```

Filters are objects. Operators start with `$`:

| SQL                                    | Mongo filter                                                                           |
| -------------------------------------- | -------------------------------------------------------------------------------------- |
| `WHERE speed > 5`                      | `{ speed: { $gt: 5 } }`                                                                |
| `WHERE boat = 'zephyr' AND speed >= 5` | `{ boat: 'zephyr', speed: { $gte: 5 } }`                                               |
| `WHERE id IN (1,2,3)`                  | `{ id: { $in: [1, 2, 3] } }`                                                           |
| `WHERE a = 1 OR b = 2`                 | `{ $or: [{ a: 1 }, { b: 2 }] }`                                                        |
| Filter on a nested field               | `{ 'ships.sog': { $gt: 10 } }` (dot notation reaches into embedded objects and arrays) |

**Sorting by `_id` gives roughly insertion order.** An `ObjectId` starts with a 4-byte creation timestamp, so `sort({ _id: -1 })` means "newest first." This repo relies on that to get the latest documents.

Mongoose specifics you'll see in this repo:

- **`.lean()`** returns plain JS objects. Without it, Mongoose returns heavier "document" objects with methods like `.save()`. Use `.lean()` for read-only APIs.
- **`__v`** is a version field Mongoose adds to documents it writes. That's why routes exclude it with `-__v`.
- **`mongoose.models['GPS'] ?? mongoose.model('GPS', schema)`** avoids registering the same model twice when Next.js hot-reloads in dev (which would throw `OverwriteModelError`).

## 5. Data modeling: embed vs reference

In Postgres you **normalize**: split data into tables and `JOIN` them back together. In Mongo the rule of thumb is: **data that's read together is stored together.**

- **Embed** when the child data belongs to the parent and is read with it. Examples here: the two batteries inside a batteries reading, every ship inside an AIS snapshot, waypoints inside a path document.
- **Reference** (store another document's `_id` or a key) when the related data is shared, large, or grows without limit. Mongo won't check that the referenced document exists. There are no foreign-key constraints or cascades.
- A single document is limited to **16 MB**, so don't embed something that grows forever (e.g. "all GPS fixes of a voyage" in one voyage document).

The current data has **no relationships at all**. Each collection is a flat time series, and nothing says which boat or voyage a reading belongs to. That's the core of the multi-voyage problem. (How to fix it belongs to the alternative-designs phase.)

## 6. Indexes

Same idea as Postgres B-tree indexes: without an index, a query scans the whole collection.

- `_id` is indexed automatically.
- Other indexes are created explicitly: `db.gps.createIndex({ timestamp: 1 })`, or declared in a Mongoose schema.
- Compound indexes work like in Postgres: field order matters.

None of the website's Mongoose models declare indexes. Whether production has any is unknown until someone checks the real database (`db.gps.getIndexes()`).

## 7. Aggregation pipeline (Mongo's `GROUP BY`)

For anything beyond find/sort/limit (grouping, averaging, reshaping, joining) Mongo uses an **aggregation pipeline**: an array of stages that documents pass through in order.

```js
// Average speed per day, roughly: SELECT day, AVG(speed) ... GROUP BY day
db.gps.aggregate([
  { $match: { speed: { $gt: 0 } } }, // WHERE
  {
    $group: { _id: { $substr: ['$timestamp', 0, 8] }, avg: { $avg: '$speed' } },
  }, // GROUP BY
  { $sort: { _id: 1 } }, // ORDER BY
]);
```

Other stages you'll see: `$project` (SELECT columns), `$lookup` (a left join), `$unwind` (turn an array into one document per element), `$limit`. This repo doesn't currently use aggregations.

## 8. Transactions and consistency

- **A write to a single document is atomic**, including all its nested fields. That's one reason embedding is popular: one document can often do what would need a transaction in Postgres.
- Multi-document transactions exist (they need a replica set; Atlas always has one), but they're slower and rarely needed for append-only telemetry like ours.

## 9. Mongoose compared with Drizzle

|                    | Drizzle + Postgres                 | Mongoose + MongoDB                                                                                        |
| ------------------ | ---------------------------------- | --------------------------------------------------------------------------------------------------------- |
| Schema definition  | `pgTable('gps', { ... })`          | `new mongoose.Schema({ ... })` + `mongoose.model(...)`                                                    |
| Schema enforced by | The database                       | Only Mongoose, and only on writes through it                                                              |
| Migrations         | drizzle-kit generates SQL          | None. Write your own scripts if old documents need changing                                               |
| TypeScript types   | Inferred from the table definition | Written by hand (`interface GPS extends mongoose.Document`) and can drift from the schema                 |
| Query style        | SQL-like builder                   | Chained `find().sort().limit()`                                                                           |
| Relations          | `relations()`, real joins          | `populate()` (an extra query per reference) or `$lookup`                                                  |
| Connection         | A pool created once                | `mongoose.connect()`, cached on `global` (`lib/mongodb.ts`) so dev hot-reloads don't open new connections |

## 10. Tools and environments

- **Local dev:** `npm run db:start` runs MongoDB in Docker (`.devcontainer/website/docker-compose.mongodb.yml`). Connection string: `mongodb://localhost:27017/sailbot_db`.
- **Production:** `.env.production` is a template with a `mongodb+srv://...` URI. That format is typical of **MongoDB Atlas** (hosted Mongo), but the real value lives in GitHub secrets / on the server, so treat "prod is Atlas" as unverified.
- **Exploring data:** `mongosh` (the CLI shell) or **MongoDB Compass** (a GUI, like pgAdmin or TablePlus). Useful first commands: `show dbs`, `use sailbot_db`, `show collections`, `db.gps.findOne()`, `db.gps.countDocuments()`, `db.gps.getIndexes()`.
- **Fake data:** `npm run simulation` runs `tests/simulation/simulation.py` to fill the local DB.

## 11. Summary

- Collections hold JSON-like documents. Nothing enforces their shape, so **the writers decide what's really in the database**. In this repo the writers are C++, Node, and Python, not the website.
- `models/*.ts` are the website's assumptions about the data, not guarantees. Check them against `sailbot_db.cpp` and the real data.
- Time is stored as strings in two different formats, and there are no indexes.
- There's no boat or voyage field anywhere.
