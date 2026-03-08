from pprint import pprint
class Phone:     #“给定制作手机需要提供的信息模板
    os = "ios"    #类属性，默认所有实例的系统。但优先级次于实例属性
    def __init__(self,os,number,is_waterproof):   #self 是物件object（元素）本身    #报错一次，原因是init写成了int
        self.os = os
        self.number = number
        self.is_waterproof = is_waterproof

""""
实例属性：self.os = os   初始化的是实例属性
os：形参
ios：实参
"""
phone1 = Phone("ios",123,True)     #类似于”给定信息，工厂制作中）
print(phone1.os)   #.可理解为“的”
pprint(vars(phone1))   #打印全体变量的另一种更为美观的方法。输出变量的呈现按照字母顺序排序而非定义


phone2 = Phone("andriod",456,False) 
phone2.color = "Red" 
print(phone2.number,phone2.is_waterproof) 
print(phone2.__dict__)    #.__dict__可打印该实例所有属性（变量）。  据输出可看出实例属性的初始化andriod高于类属性ios

""""
初始化
初始化（通常）： def __init__(self,...)    
特殊情况在python里也可以外部直接初始化，比如：phone1.color = "Red"  # 在外面突然加了一个“颜色”属性
变量是否需要初始化?
凡是需要跨越不同函数被反复读取、或者需要跨越时间被‘记住’的状态量，必须挂在 self 下初始化；
凡是只在当下运算中充当‘工具人’的临时量，随用随弃不留痕迹。
简而言之，贯穿始终的本质属性必须初始化，只用于计算的临时变量不能初始化。
"""