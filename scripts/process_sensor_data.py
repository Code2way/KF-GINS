import pandas as pd
def extract_sensor_columns(df):
    # 需要提取的列名
    columns_to_extract = [
        'timestamp',
        'Gyro.x', 'Gyro.y', 'Gyro.z',  # 陀螺仪数据
        'Gsns.x', 'Gsns.y', 'Gsns.z'   # 加速度传感器数据
    ]

    # 检查所需列是否都存在
    missing_cols = [col for col in columns_to_extract if col not in df.columns]
    if missing_cols:
        raise ValueError(f"Missing required columns: {missing_cols}")

    # 只保留需要的列
    df_selected = df[columns_to_extract]

    return df_selected
# 读取CSV文件
df = pd.read_csv('/mnt/d/GitHub/KF-GINS/bin/250227080530_000_Sensor.csv')

# 按timestamp分组并计算平均值
df_merged = df.groupby('timestamp').mean().reset_index()


df_selected = extract_sensor_columns(df_merged)
# 对Gyro数据和Gsns数据分别乘以某个数
gyro_multiplier = 0.1  # 假设Gyro数据需要乘以0.1
gsns_multiplier = 9.8  # 假设Gsns数据需要乘以9.8

df_selected[['Gyro.x', 'Gyro.y', 'Gyro.z']] = df_selected[['Gyro.x', 'Gyro.y', 'Gyro.z']] / 262.14* 0.1 * 3.14 /180 # dps
df_selected[['Gsns.x', 'Gsns.y', 'Gsns.z']] = df_selected[['Gsns.x', 'Gsns.y', 'Gsns.z']] / 16384 *9.8 * 0.1 # m/s^2
df_selected[['timestamp']] = df_selected[['timestamp']] / 1000  # 将时间戳转换为秒
# 交换 Gyro 的 x、y 轴并将 z 轴取反
df_selected[['Gyro.x', 'Gyro.y']] = df_selected[['Gyro.y', 'Gyro.x']]
df_selected['Gyro.z'] = -df_selected['Gyro.z']

# 交换 Gsns 的 x、y 轴并将 z 轴取反
df_selected[['Gsns.x', 'Gsns.y']] = df_selected[['Gsns.y', 'Gsns.x']]
df_selected['Gsns.z'] = -df_selected['Gsns.z']


# df = extract_sensor_columns(your_dataframe)
# 保存处理后的CSV文件，不包含列名
output_file = '/mnt/d/GitHub/KF-GINS/bin/Sensor_merged.txt'
df_selected.to_csv(output_file, index=False, header=False, sep=' ')

print(f"处理完成，结果保存至: {output_file}")