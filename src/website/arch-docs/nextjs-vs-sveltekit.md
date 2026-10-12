<!-- markdownlint-disable MD013 -->

# Next.js and React for SvelteKit developers

A reference for understanding the POLARIS website (`src/website`) when you already know SvelteKit. Part 1 covers React, which Next.js is built on. Part 2 covers Next.js itself. Part 3 describes how this repo actually uses them.

> **Two routers:** Next.js has an older **Pages Router** (`pages/` directory, `getServerSideProps`, `pages/_app.tsx`) and a newer **App Router** (`app/` directory). **This repo uses the App Router** (Next 15, React 18). Tutorials online mix the two up, so ignore anything that mentions `getServerSideProps`.

## Part 1: React compared with Svelte

### Components are functions that re-run

A Svelte component is a file. The compiler works out which variables are reactive and updates only the parts of the page that depend on them.

A React component is a **plain function that returns markup (JSX)**. When its state or props change, React **runs the whole function again**. It then compares the new output with the previous output and updates the page where they differ.

```tsx
// React
function Counter({ start }: { start: number }) {
  // props = function arguments
  const [count, setCount] = useState(start); // state
  return <button onClick={() => setCount(count + 1)}>{count}</button>;
}
```

```svelte
<!-- Svelte 5 equivalent -->
<script> let { start } = $props(); let count = $state(start); </script>
<button onclick={() => count++}>{count}</button>
```

Because the function re-runs, two rules follow:

- **You never change state directly.** You call the setter (`setCount(...)`). Assigning `count = 5` does nothing.
- **Code in the function body runs on every render**, not once like a Svelte `<script>` block. Anything that should run once, or only when certain values change, goes inside a hook.

### Hooks (functions starting with `use`)

| React                   | Svelte equivalent                        | Purpose                                                                          |
| ----------------------- | ---------------------------------------- | -------------------------------------------------------------------------------- |
| `useState`              | `let x` (Svelte 4) / `$state` (Svelte 5) | Local reactive state                                                             |
| `useEffect(fn, [deps])` | `onMount` / `$:` / `$effect`             | Side effects after rendering. `[]` means "run once on mount"                     |
| `useMemo`               | `$:` derived / `$derived`                | Cache a computed value                                                           |
| `useContext`            | `getContext` / `setContext`              | Pass data down the tree without passing props through every level                |
| `useRef`                | `bind:this`                              | Hold a reference to a DOM element, or a value that shouldn't trigger a re-render |

Example from this repo: `views/components/Dashboard/Dashboard.tsx` uses `useState` for the map/data toggle on mobile. It uses `useEffect(..., [])` once on mount to read `window.location.hash`, which is effectively `onMount`.

### Global state

Svelte has stores built in. React has nothing comparable built in beyond Context, so apps add a library. **This repo uses Redux, with Redux-Saga for side effects** (see `lib/redux/` and `stores/`). For now, think of it as one big global store that components subscribe to.

## Part 2: Next.js compared with SvelteKit

### File-based routing

| SvelteKit                          | Next.js App Router                                                               | In this repo                   |
| ---------------------------------- | -------------------------------------------------------------------------------- | ------------------------------ |
| `src/routes/+page.svelte`          | `app/page.tsx`                                                                   | `app/page.tsx` (dashboard)     |
| `src/routes/download/+page.svelte` | `app/download/page.tsx`                                                          | exists                         |
| `+layout.svelte`                   | `layout.tsx` (receives `children` instead of `<slot/>` / `{@render children()}`) | `app/layout.tsx`               |
| `+server.ts` (API endpoint)        | `route.ts`, exporting `GET`, `POST`, etc.                                        | `app/api/*/route.ts`           |
| `+error.svelte`                    | `error.tsx` (must be a client component)                                         | not used                       |
| Loading UI with `{#await}`         | `loading.tsx` (shown automatically while the page loads)                         | not used                       |
| `[slug]` dynamic segment           | `[slug]` (identical)                                                             | not used                       |
| `(group)` route groups             | `(group)` (identical)                                                            | not used                       |
| `src/lib` + `$lib` alias           | No convention. `@/` alias set in `tsconfig.json`                                 | `@/lib`, `@/stores`, `@/views` |

The endpoints look almost identical:

```ts
// Next.js: app/api/gps/route.ts
export async function GET(request: Request) { return NextResponse.json({ ... }); }

// SvelteKit: src/routes/api/gps/+server.ts
export async function GET({ url }) { return json({ ... }); }
```

In Next.js, only files with the special names (`page`, `layout`, `route`, `error`, `loading`, ...) create routes. Other files inside `app/` are just modules. This repo keeps its components outside `app/` anyway, in `views/components/`.

### Middleware compared with `hooks.server.ts`

| SvelteKit `handle` hook                           | Next.js `middleware.ts`                                                                                                                                         |
| ------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Runs on every server request                      | Runs only on requests that match `config.matcher`                                                                                                               |
| Full Node environment; can talk to the DB         | Historically ran in the restricted **Edge runtime** (no DB drivers, a subset of Node APIs). Newer versions can run on Node, but it is still meant to stay light |
| `event.locals` passes data down to load functions | **No `locals` equivalent.** You can redirect, rewrite, set headers/cookies, or return a response early                                                          |
| Typical use: auth, setting up the DB connection   | Typical use: auth redirects, rewrites, headers                                                                                                                  |

In this repo, `middleware.ts` does one thing: if `DISABLE_API=true`, every `/api/*` request returns `{ success: true, data: [] }`. It's a dev switch for running the frontend without a database (`npm run web:dev:no-api`).

Because there's no `locals`, things like "get a DB connection" happen inside each route instead. That's why every `route.ts` calls `await ConnectMongoDB()` itself, and `lib/mongodb.ts` caches the connection on `global` so repeated calls are cheap.

### Data loading: the biggest difference

Next.js has **no `load` function**. To understand what replaces it, you need one concept: **Server Components vs Client Components**.

**Server Components** are the default in the App Router. Every component is one unless marked otherwise.

- It runs **only on the server** and sends rendered HTML to the browser. Its JavaScript never ships to the browser.
- It **can be an `async` function and fetch data or query the DB directly** inside the component. That is Next's replacement for `load`.
- It **can't** use state, effects, event handlers, or browser APIs.

```tsx
// Server Component: the "load function" is inline
export default async function VoyagePage({ params }) {
  const voyage = await db.voyages.findOne({ id: params.id }); // runs on the server
  return <VoyageView voyage={voyage} />;
}
```

**Client Components** have `'use client'` at the top of the file. That's the "normal React" you know: it can use hooks and event handlers, and its code ships to the browser. It is _still server-rendered once_ for the first HTML, then hydrated in the browser, same as a SvelteKit page with SSR on.

Rules for mixing them:

- Server Components can render Client Components.
- Client Components can't import Server Components, but they can receive one as `children`.
- Props passed from a server component to a client component must be serializable (no functions, class instances, etc.).

| SvelteKit                                           | Next.js                                                                                                                        |
| --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `+page.server.ts` `load` (server-only data)         | `async` Server Component that queries directly                                                                                 |
| `+page.ts` universal `load`                         | No direct equivalent. Fetch in a Server Component, or in the browser from a Client Component                                   |
| `data` prop passed into `+page.svelte`              | The Server Component passes props to its child components                                                                      |
| Every `.svelte` component is interactive            | Only `'use client'` components are interactive                                                                                 |
| Form actions (`actions` in `+page.server.ts`)       | **Server Actions**: functions marked `'use server'` that a form or client component can call                                   |
| `export const ssr = false`                          | `next/dynamic(() => import(...), { ssr: false })`, applied to a single component                                               |
| `export const prerender = true` / dynamic rendering | Next tries to render pages statically at build time. `export const dynamic = 'force-dynamic'` makes it render on every request |
| `invalidate()` / `invalidateAll()`                  | `router.refresh()`, `revalidatePath()`                                                                                         |

## Part 3: How this repo uses all this

- `app/page.tsx` and `app/download/page.tsx` are **Server Components**, but they **don't load any data**. They just render `<Header />` plus `<Dashboard />` or `<Download />`.
- `app/layout.tsx` wraps everything in `<Providers>` (`lib/providers.tsx`, a `'use client'` file), which provides the Redux store to the whole app.
- `Dashboard`, `Map`, `Stats`, and the other components in `views/components/` are **`'use client'`**.
- `views/components/Map/Map.tsx` loads `Maps.tsx` with `dynamic(..., { ssr: false })`, because Leaflet needs `window` and would crash during server rendering.
- **All data is fetched in the browser.** Redux-Saga loops (`stores/*/…Sagas.ts`) call `/api/gps` and the other endpoints, then sleep for `NEXT_PUBLIC_POLLING_TIME_MS` and repeat. The server sends an empty page, and the browser fills it in by calling the API routes.

So the site uses Next.js mostly as **a React single-page app plus an API server in one deployment**. It doesn't use Server Components for data loading. In SvelteKit terms: no `load` functions anywhere, every page fetching from `+server.ts` endpoints inside `onMount`, and a `setInterval` re-fetching forever. That's a legitimate choice for a polling dashboard. But it also means `export const dynamic = 'force-dynamic'` in `app/page.tsx` has little effect, because the page has nothing server-side to make dynamic.

## Cheat sheet

| You want to...           | SvelteKit                | Next.js (App Router)                            |
| ------------------------ | ------------------------ | ----------------------------------------------- |
| Add a page               | `routes/x/+page.svelte`  | `app/x/page.tsx`                                |
| Share a layout           | `+layout.svelte`         | `layout.tsx`                                    |
| Load data on the server  | `+page.server.ts` `load` | `async` Server Component                        |
| Make an API endpoint     | `+server.ts`             | `route.ts`                                      |
| Handle a form submission | Form actions             | Server Actions                                  |
| Intercept every request  | `hooks.server.ts`        | `middleware.ts` (lighter, no `locals`)          |
| Opt out of SSR           | `ssr = false`            | `'use client'` + `dynamic(..., { ssr: false })` |
| Component state          | `$state` / `let`         | `useState`                                      |
| Run code on mount        | `onMount`                | `useEffect(() => { ... }, [])`                  |
| Global state             | Stores                   | Context, or a library such as Redux             |
