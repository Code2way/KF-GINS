import csv
import re

def parse_gnss_log(input_file, output_file):
    data = []
    current_timestamp = None
    lat = lon = alt = None
    lat_std = lon_std = alt_std = None
    hour = minute = second = millisec = None

    with open(input_file, 'r') as f:
        for line in f:
            line = line.strip()

            # 处理 HEAD 行获取时间戳
            if line.startswith('#HEAD'):
                parts = line.split(',')
                if len(parts) >= 3:
                    current_timestamp = str(float(parts[2]) / 1000)

            # 处理 GGA 行获取经纬度、高度和UTC时间
            elif line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                parts = line.split(',')
                if len(parts) >= 10:
                    try:
                        # 获取UTC时间并拆分
                        if parts[1]:
                            time_str = parts[1]
                            if len(time_str) >= 6:
                                hour = int(time_str[:2])
                                minute = int(time_str[2:4])
                                second = int(time_str[4:6])
                                millisec = int(float(time_str[6:]) * 1000) if len(time_str) > 6 else 0
                            else:
                                hour = minute = second = millisec = 0
                        else:
                            hour = minute = second = millisec = 0

                        # 转换纬度
                        if parts[2] and parts[3]:
                            lat_deg = float(parts[2][:2])
                            lat_min = float(parts[2][2:])
                            lat = lat_deg + lat_min/60
                            if parts[3] == 'S':
                                lat = -lat
                        else:
                            lat = 0
                        # 转换经度
                        if parts[4] and parts[5]:
                            lon_deg = float(parts[4][:3])
                            lon_min = float(parts[4][3:])
                            lon = lon_deg + lon_min/60
                            if parts[5] == 'W':
                                lon = -lon
                        else:
                            lon = 0
                        # 转换高度
                        if parts[9]:
                            alt = float(parts[9])
                        else:
                            alt = 0
                    except (ValueError, IndexError):
                        continue

            # 处理 GST 行获取标准差
            elif line.startswith('$GPGST') or line.startswith('$GNGST'):
                parts = line.split(',')
                if len(parts) >= 8:
                    try:
                        lat_std = float(parts[6]) if parts[6] else 0.0
                        lon_std = float(parts[7]) if parts[7] else 0.0
                        alt_std = float(parts[8].split('*')[0]) if len(parts) > 8 and parts[8] else 0.0

                        # 如果所有需要的数据都已获取，添加到结果列表
                        if all(v is not None for v in [current_timestamp, lat, lon, alt, lat_std, lon_std, alt_std]):
                            data.append([
                                current_timestamp,
                                lat,
                                lon,
                                alt,
                                lat_std,
                                lon_std,
                                alt_std,
                                hour,      # 时
                                minute,    # 分
                                second,    # 秒
                                millisec   # 毫秒
                            ])
                            # 重置变量
                            current_timestamp = lat = lon = alt = lat_std = lon_std = alt_std = None
                            hour = minute = second = millisec = None
                    except (ValueError, IndexError):
                        continue

    # 保存为CSV文件
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f, delimiter=' ')
        # 添加表头
        # writer.writerow(['Timestamp', 'Latitude', 'Longitude', 'Altitude',
        #                 'Lat_std', 'Lon_std', 'Alt_std',
        #                 'Hour', 'Minute', 'Second', 'Millisecond'])
        writer.writerows(data)

# 使用函数
input_file = '/mnt/d/GitHub/KF-GINS/bin/250227080530_000_Gnss.log'  # 输入文件名
output_file = '/mnt/d/GitHub/KF-GINS/bin/gnss_data.txt'  # 输出文件名
parse_gnss_log(input_file, output_file)