# Pion Current baseline9999

Completed10000 fresh updates,6000env,seed42. Original training env and agent
snapshots accompany this file. The checkpoint carries current_joint_position
with scale0.25 and policy-step anchoring. Export reorders controller histories
into Pion order and actions back, with embedded clipping10. The manifest above
this directory identifies exact filenames, hashes and native actor parity.
The privileged critic is omitted from deployment. This run predates the new
continuous success score; use its saved env.yaml rather than today's source to
interpret training metrics.
