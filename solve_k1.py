# 参考数值，算一下就行，可能用处不大
AIR_C_d = 0.1
AIR_ROU = 1.293

BULLET_D = 0.0425
BULLET_M = 0.041

BULLET_S = (3.14 * (BULLET_D / 2) * (BULLET_D / 2))


AIR_K0 = AIR_C_d * AIR_ROU * BULLET_S * (1/2)
print(AIR_K0)
AIR_K1 = (AIR_K0 / BULLET_M)
print(AIR_K1)
