from math import atan2, asin, sqrt

M_PI=3.1415926535

class Logger:
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        self.filename = filename

        with open(self.filename, 'w') as file:
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            vals_str=""

            # TODO Part 5: Write the values from the list to the file
            for value in values_list:
                # 处理不同类型的数据
                if isinstance(value, (list, tuple)):
                    # 如果是列表或元组，转换为字符串
                    vals_str += '"' + ';'.join(map(str, value)) + '"'
                else:
                    # 单个值直接转换为字符串
                    vals_str += str(value)
                vals_str += ", "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        # 文件已经在log_values中自动保存，这里可以添加额外的保存逻辑
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:
            # 读取表头
            if not read_headers:
                header_line = file.readline().strip()
                values = header_line.split(',')
                
                for val in values:
                    if val == '' or val == '\n':
                        break
                    # 去除可能的引号和空格
                    clean_val = val.strip().strip('"\'')
                    if clean_val:
                        headers.append(clean_val)
                
                read_headers = True
            
            # 读取数据行
            for line in file:
                line = line.strip()
                if not line:  # 跳过空行
                    continue
                    
                values = line.split(',')
                row = []                
                
                for val in values:
                    if val == '' or val == '\n':
                        break
                    
                    clean_val = val.strip().strip('"\'')
                    if not clean_val:
                        continue
                        
                    try:
                        # 尝试转换为浮点数
                        if ';' in clean_val:
                            # 如果是分号分隔的数据，转换为浮点数列表
                            num_list = [float(x) for x in clean_val.split(';')]
                            row.append(num_list)
                        else:
                            row.append(float(clean_val))
                    except ValueError:
                        # 如果转换失败，保留字符串
                        row.append(clean_val)

                if row:  # 只添加非空行
                    table.append(row)
        
        return headers, table


# TODO Part 5: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x, y, z, w = quat
    
    # 计算偏航角 (yaw) - 绕Z轴旋转
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    
    # 计算俯仰角 (pitch) - 绕Y轴旋转
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        # 使用90度如果超出范围
        pitch = M_PI / 2 if sinp >= 0 else -M_PI / 2
    else:
        pitch = asin(sinp)
    
    # 计算滚转角 (roll) - 绕X轴旋转
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)
    
    return roll, pitch, yaw

# 为了方便使用，添加一个只返回yaw的函数
def yaw_from_quaternion(quat):
    """
    Convert quaternion to yaw angle only.
    quat = [x, y, z, w]
    """
    x, y, z, w = quat
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    return yaw

# 添加四元数到欧拉角转换的另一种实现（使用atan2）
def quaternion_to_euler(x, y, z, w):
    """
    Alternative implementation that takes separate quaternion components.
    """
    # 滚转 (x轴旋转)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)
    
    # 俯仰 (y轴旋转)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = M_PI / 2 if sinp >= 0 else -M_PI / 2
    else:
        pitch = asin(sinp)
    
    # 偏航 (z轴旋转)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw
