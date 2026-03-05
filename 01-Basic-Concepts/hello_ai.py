import requests
import os

# 💡 核心操作：告诉 Python 别用系统代理，直接联网
os.environ['NO_PROXY'] = 'httpbin.org'

print("正在尝试（直连）连接 AI 学习服务器...")

try:
    # 增加 verify=False 可以强制跳过证书检查（虽然不推荐，但在调试环境下很管用）
    response = requests.get('https://httpbin.org/ip', timeout=5)
    ip = response.json()['origin']
    print(f"✅ 联网成功！")
    print(f"游奕，你的 Python 助手已上线，当前网络位置：{ip}")
except Exception as e:
    print(f"❌ 联网依然失败，原因：{e}")

user_name = input("请输入你的名字：")
print(f"你好 {user_name}，这是你第一个联网的 AI 脚本！")