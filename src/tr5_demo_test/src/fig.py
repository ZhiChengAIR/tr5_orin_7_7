""" import pandas as pd
import matplotlib.pyplot as plt

df_mjc = pd.read_csv("/home/ubantu/ZCAI/TR5/tr5_local_test/tr5_control_local/build/mujoco_log.csv")
df_mock = pd.read_csv("/home/ubantu/ZCAI/TR5/tr5_local_test/tr5_control_local/build/mock_log.csv")

N = min(len(df_mjc), len(df_mock), 3000)

time = df_mjc['time'].to_numpy()[:N]
tau_mjc = df_mjc['tau_0'].to_numpy()[:N]
tau_mock = df_mock['tau_0'].to_numpy()[:N]

plt.figure(figsize=(10, 5))
plt.plot(time, tau_mjc, label='Mujoco - tau_0', linewidth=2)
plt.plot(time, tau_mock, label='Mock   - tau_0', linewidth=2, linestyle='--')

plt.title('Torque Comparison: tau_0')
plt.xlabel('Time (s)')
plt.ylabel('Torque (Nm)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show() """

""" import pandas as pd
import matplotlib.pyplot as plt

# 替换为你自己的路径
csv_file = "/home/ubantu/ZCAI/TR5/tr5_local_test/tr5_control_local/build/mujoco_log.csv"

# 加载数据
df = pd.read_csv(csv_file)

# 要对比的关节编号
joint_indices = [0]

# 画图
plt.figure(figsize=(12, 6))

for idx in joint_indices:
    tau_col = f"tau_{idx}"
    tor_des_col = f"motor_tor_des[{idx}]" if f"motor_tor_des[{idx}]" in df.columns else f"tor_des_{idx}"

    if tau_col not in df.columns or tor_des_col not in df.columns:
        print(f"⚠️ 缺失列：{tau_col} 或 {tor_des_col}，跳过关节 {idx}")
        continue

    time = df["time"].to_numpy()
    tau = df[tau_col].to_numpy()
    tor_des = df[tor_des_col].to_numpy()

    plt.plot(time, tau, label=f"motors_tor_out[{idx}]", linewidth=1.5)
    plt.plot(time, tor_des, '--', label=f"wbc_tauJointRes[{idx}]", linewidth=1.2)

plt.title("Torques: motors_tor_out vs wbc_tauJointRes")
plt.xlabel("Time [s]")
plt.ylabel("Torque [Nm]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show() """

import pandas as pd
import matplotlib.pyplot as plt
import os

# === 文件路径 ===
mujoco_path = "/home/zjy/tr5_control_local/build/mujoco_log.csv"
mock_path = "/home/zjy/tr5_test/mock_log.csv"

if not os.path.exists(mujoco_path):
    print("❌ CSV文件路径错误 111")
    exit()
if not os.path.exists(mock_path):
    print("❌ CSV文件路径错误 222")
    exit()
# === 加载CSV ===
if not os.path.exists(mujoco_path) or not os.path.exists(mock_path):
    print("❌ CSV文件路径错误")
    exit()

df_mujoco = pd.read_csv(mujoco_path)
df_mock = pd.read_csv(mock_path)

# === 对比哪些关节 ===
joint_indices = list(range(20))  # 比较前6个关节

plt.figure(figsize=(14, 8))
# for idx in joint_indices:
#     col_name = f"wbc_tauJointRes_{idx}"
#     if col_name not in df_mujoco.columns or col_name not in df_mock.columns:
#         print(f"⚠️ 缺失列 {col_name}，跳过")
#         continue

#     plt.plot(df_mujoco["time"].values, df_mujoco[col_name].values, label=f"Mujoco [{col_name}]")
#     plt.plot(df_mock["time"].values, df_mock[col_name].values, '--', label=f"Mock   [{col_name}]")

col_name = f"tau_{5}"
if col_name not in df_mujoco.columns or col_name not in df_mock.columns:
    print(f"⚠️ 缺失列 {col_name}，跳过")

plt.plot(df_mujoco["time"].values, df_mujoco[col_name].values, label=f"Mujoco [{col_name}]")
plt.plot(df_mock["time"].values, df_mock[col_name].values, '--', label=f"Mock   [{col_name}]")


plt.title("Compare motor_tor_des from Mujoco vs Mock")
plt.xlabel("Time [s]")
plt.ylabel("Torque [Nm]")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()