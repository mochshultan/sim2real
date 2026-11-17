# Pion Current MuJoCo plant

Copied from `/home/shultan/IsaacLab/sim2sim_mujoco/pion_qr1/models/pion_qr1.xml`
and its referenced meshes. The source converter used Pion's URDF and training
USD collider overlay. All mesh paths are now package-local. Joint armature0.01
and zero passive damping/friction follow the saved baseline actuator config.
No policy, gains, reference transform or inference code lives in the model.
The bridge applies the incoming deployment-controller gains and current targets.
