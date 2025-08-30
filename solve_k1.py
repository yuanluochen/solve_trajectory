# 参考数值，算一下就行，可能用处不大
# k1参考数值，来源于TUP的开源，大弹丸k = 0.00556，发光大弹丸k = 0.00530，小弹丸k = 0.01903。
AIR_C_d = 0.1
AIR_ROU = 1.293

BULLET_D = 0.0425
BULLET_M = 0.041

BULLET_S = (3.14 * (BULLET_D / 2) * (BULLET_D / 2))


AIR_K0 = AIR_C_d * AIR_ROU * BULLET_S * (1/2)
print(AIR_K0)
AIR_K1 = (AIR_K0 / BULLET_M)
print(AIR_K1)
