# Release notes format

The changelogs are the reference; release notes are the announcement. A reader should learn,
from the first screen, what they can fly now, whether their code or tune breaks, and what to
do. Every claim comes from a changelog bullet or the root README **Status**; nothing new.

## Umbrella release (`stable-flight` tag)

Title `stable-flight x.y.z`. Sections in this order:

1. **Lead:** one sentence on what a user can now fly or do, only what has flown (README
   **Status**). A first release says what the crate is.
2. **Hero image** (optional): a `just plot` of a flight this release enables, captioned with
   the airframe, simulator, and what was flown. Upload it to the release; no GIF per feature.
3. **`[!WARNING]`**, only if any **Breaking (behavior):** bullets: those changes in plain words
   and what to do (re-run autotune, or pin a param to its old value). At most one more alert,
   a `[!NOTE]` for a new crate.
4. **Highlights:** grouped under `###` by airframe or capability (copter, plane, VTOL, tuning,
   link and ground station). Each bullet: bold feature name, what it does for the user, issue
   at the end: `**Plane waypoints:** hold altitude and airspeed by total energy (#n)`. No type
   names, no evidence; those stay in the changelog.
5. **Upgrading:** a table grouping the **Breaking** bullets by who they hit, then a task list
   of steps. No **Breaking** marks repeated elsewhere in the notes.
6. **Crates:** one row per published crate, its version (**new** on a first release), role,
   and links.
7. **Full changelogs:** each crate's changelog section verbatim inside `<details>`.

No emoji headings, no badge rows, no headings below `###`.

```markdown
<Lead sentence.>

![<Airframe, simulator, what was flown>](<uploaded plot>)

> [!WARNING]
> **Flies differently after upgrading.** <Behavior changes in plain words.> <What to do.>

## Highlights

### <Airframe or capability>
- **<Feature>:** <what it does for the user> (#n)

## Upgrading

| If you… | What changed | Do this |
|---|---|---|
| fly a tuned copter | <…> | <…> |
| implement a backend | <…> | <…> |
| build configs with struct literals | <new fields> | add `..Default::default()` |

- [ ] <step>

## Crates

| Crate | Version | Role | Links |
|---|---|---|---|
| `stable-flight` | x.y.z | flight control core | [crates.io](https://crates.io/crates/stable-flight) · [docs](https://docs.rs/stable-flight/x.y.z) · [changelog](https://github.com/wboayue/stable-flight/blob/stable-flight-vx.y.z/crates/stable-flight/CHANGELOG.md) |

<details><summary><b>stable-flight</b> x.y.z changelog</summary>

<the crate's changelog section>

</details>
```

## Thin release (every other tag)

Title `<crate> x.y.z`.

```markdown
<Crate role in one line.> This release: <the one or two changes that matter for this crate>.

**Full notes:** [stable-flight x.y.z](https://github.com/wboayue/stable-flight/releases/tag/stable-flight-vx.y.z) · [crates.io](https://crates.io/crates/<crate>) · [docs](https://docs.rs/<crate>/x.y.z) · [changelog](https://github.com/wboayue/stable-flight/blob/<crate>-vx.y.z/crates/<crate>/CHANGELOG.md)
```
