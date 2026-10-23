import numpy as np
np.set_printoptions(precision=4, suppress=True, linewidth=110)
R, lx, ly = 0.05, 0.10, 0.10; k = lx + ly
J_IK = (1/R)*np.array([[1,-1,-k],[1,1,k],[1,1,-k],[1,-1,k]])      # rows: FL, FR, RL, RR
J_FK = np.linalg.pinv(J_IK)
J_FK_closed = (R/4)*np.array([[1,1,1,1],[-1,1,1,-1],[-1/k,1/k,-1/k,1/k]])
print(f"$ python kinematics_check.py      # R={R} m, lx={lx} m, ly={ly} m, k=lx+ly={k} m   (wheel order FL, FR, RL, RR)")
print("J_IK (rad/s per [m/s, m/s, rad/s]) =");  print(J_IK)
print("J_FK from pinv(J_IK) =");                print(J_FK)
print("J_FK closed form      =");               print(J_FK_closed)
print("max |pinv - closed form|        =", np.abs(J_FK-J_FK_closed).max())
print("J_FK @ J_IK (should be I3) =");          print(J_FK@J_IK)
print("J_IK @ J_FK (rank-3 projector, != I4) ="); print(J_IK@J_FK)
n = np.array([1,1,-1,-1])/2
print("null-space direction n = (FL,FR,RL,RR) = (1,1,-1,-1)/2   -> J_FK @ n =", J_FK@n, " (no body motion)")
print("singular values of J_IK:", np.linalg.svd(J_IK, compute_uv=False))
rng = np.random.default_rng(1); worst = 0
for _ in range(1000):
    v = rng.normal(size=3); worst = max(worst, np.abs(J_FK@(J_IK@v) - v).max())
print("round trip v -> wheels -> v, worst error over 1000 random twists:", worst)
print("\npure motions  [vx vy wz] -> [w_fl w_fr w_rl w_rr] (rad/s)")
for name, v in (("forward 0.3 m/s", [0.3,0,0]), ("strafe left 0.3 m/s", [0,0.3,0]), ("yaw CCW 1 rad/s", [0,0,1.0])):
    print(f"  {name:20s}", J_IK@np.array(v))
