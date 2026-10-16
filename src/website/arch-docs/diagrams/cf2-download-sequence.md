<!-- markdownlint-disable MD013 -->

# CF2: Download a dataset

A visitor already on the downloads page clicks a tabular format (CSV or XLSX) for one dataset. GPS is used as the example; every dataset follows the same path with its own endpoint. See "Core functionality" in [`ARCHITECTURE.md`](../ARCHITECTURE.md).

```mermaid
sequenceDiagram
    autonumber
    actor V as Visitor
    participant B as Browser<br/>(downloads page)
    participant S as Next.js server<br/>(API routes)
    participant DB as MongoDB

    Note over V,DB: Already on /download. When the page loaded, it fetched<br/>each dataset's "last updated" date from /api/datasets.

    V->>B: Click CSV or XLSX on the GPS dataset
    B->>S: GET /api/gps
    S->>DB: Find every document in the gps collection
    DB-->>S: All documents
    S-->>B: JSON { success, data: [...] }

    B->>B: Flatten into rows (one per reading,<br/>or one per sensor / ship / waypoint)
    alt CSV
        B->>B: Join rows into CSV text
    else XLSX
        B->>B: Build an Excel workbook (ExcelJS)
    end
    B-->>V: Browser saves the file (e.g. gps.CSV)
```

## Takeaways

- **One click, one request, the whole collection.** There's no filtering by date range or voyage, and no paging. The file contains everything ever stored for that dataset.
- **The file is built in the browser, not on the server.** The API only returns JSON. JSON downloads skip the flattening step and save that JSON as-is.
- **Not shown:** the background polling loops from CF1 keep running on this page too, because the Redux store is shared across the whole site.
