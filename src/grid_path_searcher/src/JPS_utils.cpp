#include "JPS_utils.h"
// nsz 数组定义了不同移动类型下需要检查的邻居数量
// nsz[norm1][0]: 自然邻居 (Natural Neighbors) 的数量
// nsz[norm1][1]: 强迫邻居 (Forced Neighbors) 的数量
// norm1=0 (起点): 26个自然邻居, 0个强迫
// norm1=1 (直线): 1个自然邻居, 8个强迫
// norm1=2 (平面斜向): 3个自然邻居, 12个强迫
// norm1=3 (立体斜向): 7个自然邻居, 12个强迫
// nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}}
constexpr int JPS3DNeib::nsz[4][2];
// 构造函数：初始化查找表
JPS3DNeib::JPS3DNeib() 
{
    int id = 0;
    // 遍历所有可能的移动方向 (dx, dy, dz)，每个分量取值为 -1, 0, 1
    // 3*3*3 = 27 种情况 (包括静止 0,0,0)
    for(int dz = -1; dz <= 1; ++ dz) {
        for(int dy = -1; dy <= 1; ++ dy) {
            for(int dx = -1; dx <= 1; ++ dx) {
                // 计算移动方向的 L1 范数 (曼哈顿距离)，用于分类移动类型
                // 0: 静止, 1: 直线, 2: 平面斜向, 3: 立体斜向
                //norm1也就是这个节点的方向
                int norm1 = abs(dx) + abs(dy) + abs(dz);
                // 1. 填充自然邻居表 (ns)
                // 也就是如果没有障碍物，JPS 规定需要继续探索的方向
                for(int dev = 0; dev < nsz[norm1][0]; ++ dev)
                    Neib(dx,dy,dz,norm1,dev, ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);
                // 2. 填充强迫邻居表 (f1, f2)
                // f1: 障碍物的位置 (如果在这些位置检测到障碍物...)
                // f2: 对应的强迫拓展方向 (...就需要向这些方向搜索)
                for(int dev = 0; dev < nsz[norm1][1]; ++ dev){
                    FNeib(dx,dy,dz,norm1,dev,
                    f1[id][0][dev],f1[id][1][dev], f1[id][2][dev],
                    f2[id][0][dev],f2[id][1][dev], f2[id][2][dev]);
                }
                
                id ++;
            }
        }
    }
}

// 辅助函数：根据移动方向和规则生成自然邻居 (Natural Neighbors)
// 输入: dx, dy, dz (当前父节点来的方向), norm1 (移动类型), dev (邻居序号)
// 输出: tx, ty, tz (需要探索的自然邻居方向)
void JPS3DNeib::Neib(int dx, int dy, int dz, int norm1, int dev,
    int& tx, int& ty, int& tz)
{
    switch(norm1){
        case 0:
            // 情况 0: 起点 (dx=dy=dz=0)。
            // 起点没有父节点方向，需要搜索周围所有 26 个邻居。
            switch(dev){
                case 0:  tx=1;  ty=0;  tz=0;  return;
                case 1:  tx=-1; ty=0;  tz=0;  return;
                case 2:  tx=0;  ty=1;  tz=0;  return;
                case 3:  tx=1;  ty=1;  tz=0;  return;
                case 4:  tx=-1; ty=1;  tz=0;  return;
                case 5:  tx=0;  ty=-1; tz=0;  return;
                case 6:  tx=1;  ty=-1; tz=0;  return;
                case 7:  tx=-1; ty=-1; tz=0;  return;
                case 8:  tx=0;  ty=0;  tz=1;  return;
                case 9:  tx=1;  ty=0;  tz=1;  return;
                case 10: tx=-1; ty=0;  tz=1;  return;
                case 11: tx=0;  ty=1;  tz=1;  return;
                case 12: tx=1;  ty=1;  tz=1;  return;
                case 13: tx=-1; ty=1;  tz=1;  return;
                case 14: tx=0;  ty=-1; tz=1;  return;
                case 15: tx=1;  ty=-1; tz=1;  return;
                case 16: tx=-1; ty=-1; tz=1;  return;
                case 17: tx=0;  ty=0;  tz=-1; return;
                case 18: tx=1;  ty=0;  tz=-1; return;
                case 19: tx=-1; ty=0;  tz=-1; return;
                case 20: tx=0;  ty=1;  tz=-1; return;
                case 21: tx=1;  ty=1;  tz=-1; return;
                case 22: tx=-1; ty=1;  tz=-1; return;
                case 23: tx=0;  ty=-1; tz=-1; return;
                case 24: tx=1;  ty=-1; tz=-1; return;
                case 25: tx=-1; ty=-1; tz=-1; return;
            }
            break;
        case 1:
            // 情况 1: 直线移动 (例如 (1,0,0))
            // 自然邻居只有一个：继续沿当前方向前进。
            tx = dx; ty = dy; tz = dz; return;
        case 2:
            // 情况 2: 2D 平面斜向移动 (例如 (1,1,0))
            // 自然邻居有 3 个：
            // 1. 前方 (dx, dy, dz)
            // 2. 分量方向1 (dx, 0, 0)
            // 3. 分量方向2 (0, dy, 0)
            switch(dev){
                case 0:
                    if(dz == 0){
                        tx = 0; ty = dy; tz = 0; return;
                    }else{
                        tx = 0; ty = 0; tz = dz; return;
                    }
                case 1:
                    if(dx == 0){
                        tx = 0; ty = dy; tz = 0; return;
                    }else{
                        tx = dx; ty = 0; tz = 0; return;
                    }
                case 2:
                    tx = dx; ty = dy; tz = dz; return;
            }
            break;
        case 3:
            // 情况 3: 3D 立体斜向移动 (例如 (1,1,1))
            // 自然邻居有 7 个：
            // 主方向(1个) + 2D平面分量(3个) + 1D直线分量(3个)
            switch(dev){
                case 0: tx = dx; ty =  0; tz =  0; return;
                case 1: tx =  0; ty = dy; tz =  0; return;
                case 2: tx =  0; ty =  0; tz = dz; return;
                case 3: tx = dx; ty = dy; tz =  0; return;
                case 4: tx = dx; ty =  0; tz = dz; return;
                case 5: tx =  0; ty = dy; tz = dz; return;
                case 6: tx = dx; ty = dy; tz = dz; return;
            }
    }
}
// 辅助函数：根据移动方向生成强迫邻居 (Forced Neighbors) 规则
// 输入: dx, dy, dz (移动方向)
// 输出: fx, fy, fz (障碍物位置检测点), nx, ny, nz (如果检测到障碍物，需要产生的强迫拓展方向)
void JPS3DNeib::FNeib( int dx, int dy, int dz, int norm1, int dev,
                          int& fx, int& fy, int& fz,
                          int& nx, int& ny, int& nz)
{
    switch(norm1){
        case 1:
        // 情况 1: 直线移动 (例如沿 X 轴前进)
        // 需要检查周围 8 个与运动方向垂直的位置是否有障碍物。
            switch(dev){
                case 0: fx= 0; fy= 1; fz = 0; break;
                case 1: fx= 0; fy=-1; fz = 0; break;
                case 2: fx= 1; fy= 0; fz = 0; break;
                case 3: fx= 1; fy= 1; fz = 0; break;
                case 4: fx= 1; fy=-1; fz = 0; break;
                case 5: fx=-1; fy= 0; fz = 0; break;
                case 6: fx=-1; fy= 1; fz = 0; break;
                case 7: fx=-1; fy=-1; fz = 0; break;
            }
            nx = fx; ny = fy; nz = dz;// 基础强迫方向：指向障碍物方向 + 前进方向
            // switch order if different direction
            // 下面的逻辑是为了处理坐标轴旋转
            // 因为上面的 switch 是写死假设沿 X 轴移动的，如果实际是沿 Y 或 Z 轴，需要交换坐标
            if(dx != 0){
                fz = fx; fx = 0;
                nz = fz; nx = dx;
            }

            if(dy != 0){
                fz = fy; fy = 0;
                nz = fz; ny = dy;
            }
            return;
        case 2:
            // 情况 2: 2D 平面斜向移动 (例如在 XY 平面 (1,1,0))
            // 这里的逻辑非常繁琐，需要枚举所有可能的障碍物位置以及它们引发的强迫方向
            // 主要分为三类大情况：dx=0 (YZ面), dy=0 (XZ面), dz=0 (XY面)
            if(dx == 0){
                switch(dev){
                    case 0:
                        fx = 0; fy = 0; fz = -dz;
                        nx = 0; ny = dy; nz = -dz;
                        return;
                    case 1:
                        fx = 0; fy = -dy; fz = 0;
                        nx = 0; ny = -dy; nz = dz;
                        return;
                    case 2:
                        fx = 1; fy = 0; fz = 0;
                        nx = 1; ny = dy; nz = dz;
                        return;
                    case 3:
                        fx = -1; fy = 0; fz = 0;
                        nx = -1; ny = dy; nz = dz;
                        return;
                    case 4:
                        fx = 1; fy = 0; fz = -dz;
                        nx = 1; ny = dy; nz = -dz;
                        return;
                    case 5:
                        fx = 1; fy = -dy; fz = 0;
                        nx = 1; ny = -dy; nz = dz;
                        return;
                    case 6:
                        fx = -1; fy = 0; fz = -dz;
                        nx = -1; ny = dy; nz = -dz;
                        return;
                    case 7:
                        fx = -1; fy = -dy; fz = 0;
                        nx = -1; ny = -dy; nz = dz;
                        return;
                    // Extras
                    case 8:
                        fx = 1; fy = 0; fz = 0;
                        nx = 1; ny = dy; nz = 0;
                        return;
                    case 9:
                        fx = 1; fy = 0; fz = 0;
                        nx = 1; ny = 0; nz = dz;
                        return;
                    case 10:
                        fx = -1; fy = 0; fz = 0;
                        nx = -1; ny = dy; nz = 0;
                        return;
                    case 11:
                        fx = -1; fy = 0; fz = 0;
                        nx = -1; ny = 0; nz = dz;
                        return;
                }
            }
            else if(dy == 0){
                switch(dev){
                    case 0:
                        fx = 0; fy = 0; fz = -dz;
                        nx = dx; ny = 0; nz = -dz;
                        return;
                    case 1:
                        fx = -dx; fy = 0; fz = 0;
                        nx = -dx; ny = 0; nz = dz;
                        return;
                    case 2:
                        fx = 0; fy = 1; fz = 0;
                        nx = dx; ny = 1; nz = dz;
                        return;
                    case 3:
                        fx = 0; fy = -1; fz = 0;
                        nx = dx; ny = -1;nz = dz;
                        return;
                    case 4:
                        fx = 0; fy = 1; fz = -dz;
                        nx = dx; ny = 1; nz = -dz;
                        return;
                    case 5:
                        fx = -dx; fy = 1; fz = 0;
                        nx = -dx; ny = 1; nz = dz;
                        return;
                    case 6:
                        fx = 0; fy = -1; fz = -dz;
                        nx = dx; ny = -1; nz = -dz;
                        return;
                    case 7:
                        fx = -dx; fy = -1; fz = 0;
                        nx = -dx; ny = -1; nz = dz;
                        return;
                    // Extras
                    case 8:
                        fx = 0; fy = 1; fz = 0;
                        nx = dx; ny = 1; nz = 0;
                        return;
                    case 9:
                        fx = 0; fy = 1; fz = 0;
                        nx = 0; ny = 1; nz = dz;
                        return;
                    case 10:
                        fx = 0; fy = -1; fz = 0;
                        nx = dx; ny = -1; nz = 0;
                        return;
                    case 11:
                        fx = 0; fy = -1; fz = 0;
                        nx = 0; ny = -1; nz = dz;
                        return;
                }
            }
            else{// dz==0
                switch(dev){
                    case 0:
                        fx = 0; fy = -dy; fz = 0;
                        nx = dx; ny = -dy; nz = 0;
                        return;
                    case 1:
                        fx = -dx; fy = 0; fz = 0;
                        nx = -dx; ny = dy; nz = 0;
                        return;
                    case 2:
                        fx =  0; fy = 0; fz = 1;
                        nx = dx; ny = dy; nz = 1;
                        return;
                    case 3:
                        fx =  0; fy = 0; fz = -1;
                        nx = dx; ny = dy; nz = -1;
                        return;
                    case 4:
                        fx = 0; fy = -dy; fz = 1;
                        nx = dx; ny = -dy; nz = 1;
                        return;
                    case 5:
                        fx = -dx; fy = 0; fz = 1;
                        nx = -dx; ny = dy; nz = 1;
                        return;
                    case 6:
                        fx = 0; fy = -dy; fz = -1;
                        nx = dx; ny = -dy; nz = -1;
                        return;
                    case 7:
                        fx = -dx; fy = 0; fz = -1;
                        nx = -dx; ny = dy; nz = -1;
                        return;
                    // Extras
                    case 8:
                        fx =  0; fy = 0; fz = 1;
                        nx = dx; ny = 0; nz = 1;
                        return;
                    case 9:
                        fx = 0; fy = 0; fz = 1;
                        nx = 0; ny = dy; nz = 1;
                        return;
                    case 10:
                        fx =  0; fy = 0; fz = -1;
                        nx = dx; ny = 0; nz = -1;
                        return;
                    case 11:
                        fx = 0; fy = 0; fz = -1;
                        nx = 0; ny = dy; nz = -1;
                        return;
                }
            }
            break;
        case 3:
        // 情况 3: 3D 立体斜向移动 (dx, dy, dz 都不为 0)
            // 这是最复杂的情况，需要检查 12 个潜在的障碍物位置
            switch(dev){
                case 0:
                    fx = -dx; fy = 0; fz = 0;
                    nx = -dx; ny = dy; nz = dz;
                    return;
                case 1:
                    fx = 0; fy = -dy; fz = 0;
                    nx = dx; ny = -dy; nz = dz;
                    return;
                case 2:
                    fx = 0; fy = 0; fz = -dz;
                    nx = dx; ny = dy; nz = -dz;
                    return;
                // Need to check up to here for forced!
                case 3:
                    fx = 0; fy = -dy; fz = -dz;
                    nx = dx; ny = -dy; nz = -dz;
                    return;
                case 4:
                    fx = -dx; fy = 0; fz = -dz;
                    nx = -dx; ny = dy; nz = -dz;
                    return;
                case 5:
                    fx = -dx; fy = -dy; fz = 0;
                    nx = -dx; ny = -dy; nz = dz;
                    return;
                // Extras
                case 6:
                    fx = -dx; fy = 0; fz = 0;
                    nx = -dx; ny = 0; nz = dz;
                    return;
                case 7:
                    fx = -dx; fy = 0; fz = 0;
                    nx = -dx; ny = dy; nz = 0;
                    return;
                case 8:
                    fx = 0; fy = -dy; fz = 0;
                    nx = 0; ny = -dy; nz = dz;
                    return;
                case 9:
                    fx = 0; fy = -dy; fz = 0;
                    nx = dx; ny = -dy; nz = 0;
                    return;
                case 10:
                    fx = 0; fy = 0; fz = -dz;
                    nx = 0; ny = dy; nz = -dz;
                    return;
                case 11:
                    fx = 0; fy = 0; fz = -dz;
                    nx = dx; ny = 0; nz = -dz;
                    return;
            }
    }
}