# Cursor starter prompt

Use this when asking Cursor to implement the first real feature.

```text
Read .cursor/rules/, docs/MASTER_ENGINEERING_GUIDE.md, docs/STATUS.md,
docs/interfaces.md, docs/calibration.md, and docs/architecture.md.

Treat this starter source tree as the target architecture.
Treat any old/legacy source only as behavioral reference.

Goal:
<one small goal only>

Before editing:
1. Explain which package owns this responsibility.
2. List every file you plan to create/change.
3. State which interfaces and TF ownership will remain unchanged.
4. State any hardware facts you still need to verify.
5. Give a Definition of Done and exact test commands.

Rules:
- Do not preserve old monolithic structure.
- Do not invent physical constants or hardware wiring.
- Do not create a second publisher for an existing TF transform.
- Do not put business logic in warotrans_bringup.
- Do not implement future phases early.
- Prefer small, testable changes.

After editing:
1. Show the important diff conceptually.
2. Run/build available checks.
3. Give robot-side validation commands.
4. Update STATUS.md only after real evidence confirms the checkpoint.
```
