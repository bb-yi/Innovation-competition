#include "filtering.h"
#include "tool.h"

/**
 *@brief 限幅滤波法
方法：根据经验判断，确定两次采样允许的最大偏差值（设为A），每次检测到新值时判断：
如果本次值与上次值之差<=A，则本次值有效，
如果本次值与上次值之差>A，则本次值无效，放弃本次值，用上次值代替本次值。
优点：能克服偶然因素引起的脉冲干扰
缺点：无法抑制周期性的干扰，平滑度差
 *
 * @param NewValue 当前的值
 * @param lastValue 上一个值
 * @param maxError 最大偏差
 * @return float
 */
float clampFilter(float NewValue, float lastValue, float maxDifference)
{
    // 计算新值和上一次值之间的差值
    float difference = Abs(NewValue - lastValue);

    // 判断差值是否在允许的偏差范围内
    if (difference <= maxDifference)
    {
        // 如果在范围内，则本次值有效，返回新值
        return NewValue;
    }
    else
    {
        // 如果超出范围，则本次值无效，返回上一次的滤波值
        return lastValue;
    }
}

/**
 * 一阶滞后滤波器
 *一阶滞后滤波法
方法： 取a=0-1，本次滤波结果=(1-a)*本次采样值+a*上次滤波结果。
优点：  对周期性干扰具有良好的抑制作用；
适用于波动频率较高的场合。
平滑度高，适于高频振荡的系统。
缺点： 相位滞后，灵敏度低；
滞后程度取决于a值大小；
不能消除滤波频率高于采样频率1/2的干扰信号。
 * @param NewValue 当前采样值
 * @param lastValue 上次滤波结果
 * @param alpha 滤波系数，取值范围0到1
 * @return 返回滤波后的值
 */
float FirstOrderLagFilter(float NewValue, float lastValue, float alpha)
{
    // 计算本次滤波结果
    float filteredValue = (1.0f - alpha) * NewValue + alpha * lastValue;
    // 返回滤波结果
    return filteredValue;
}