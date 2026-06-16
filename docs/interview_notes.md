# Interview Notes

## Tracking Pipeline

A practical radar tracker usually contains:

1. Prediction
2. Gating
3. Data association
4. Measurement update
5. Track management
6. Output formatting

## Gating

Gating rejects unlikely track-detection pairs before association.
This reduces false associations and computational load.

## GNN

Global Nearest Neighbor solves a global assignment problem.
It is simple, deterministic, and often a strong baseline.

## JPDA

Joint Probabilistic Data Association handles ambiguous cases by updating tracks with weighted combinations of validated measurements.

## MHT

Multiple Hypothesis Tracking keeps multiple association hypotheses over time.
It is powerful but computationally expensive.

## Bernoulli Filter

A Bernoulli filter estimates:
- target state
- probability that the target exists

It is useful in clutter and missed-detection environments.
