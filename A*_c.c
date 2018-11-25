    /* 
     * A* 算法模拟 
     */  
      
    #include <stdio.h>  
    #include <stdlib.h>  
    #include <vector>  
      
    using std::vector;  
      
    typedef struct stNode  
    {  
        stNode() : x(0), y(0), px(0), py(0), f(0), g(0), h(0)  
        {}  
      
        int x;  
        int y;  
        int px; /* 父结点横坐标 */  
        int py; /* 父结点纵坐标 */  
        int f;  
        int g;  
        int h;  
    }Node;  
      
    /* 
     * 地图0为通路，1为起点，2为终点，-1为不可达 
     */  
    #define ROW 5  
    #define COLUMN 5  
    int short_map[][COLUMN] = {  
        {0, 0, 0, 0, 0},  
        {0, -1, 0, -1, 0},  
        {1, -1, 0, -1, 2},  
        {0, -1, 0, -1, 0},  
        {0, 0, 0, 0, 0}};  
      
    vector<Node> close_map;  
    vector<Node> open_map;  
      
    /* 
     * 找到本次查找的边界 
     */  
    void CalcBound(Node & cur, int &startx, int &starty, int &endx, int &endy)  
    {  
        startx = (cur.x > 0) ? (cur.x - 1) : cur.x;  
        starty = (cur.y > 0) ? (cur.y - 1) : cur.y;  
        endx = (cur.x < COLUMN) ? (cur.x + 1) : cur.x;  
        endy = (cur.y < COLUMN) ? (cur.y + 1) : cur.y;  
    }  
      
    /* 
     * 计算G值，G=从起点 A 移动到指定方格的移动代价，沿着到达该方格而生成的路径  
     */  
    int CalcG(Node cur, int nextx, int nexty)  
    {  
        if (cur.x == nextx || cur.y == nexty)  
        {  
            return 10;    
        }  
        else  
        {  
            return 14;  
        }  
    }  
      
    /* 
     * 计算H值，H=从指定的方格移动到终点 B 的估算成本 
     * 我们使用 Manhattan 方法，计算从当前方格横向或纵向移动到达目标所经过的方格数，忽略对角移动，然后把总数乘以 10  
     * 
     */  
    int CalcH(Node dst, int nextx, int nexty)  
    {  
        int stepnum = 0;  
      
        stepnum += ((dst.x >= nextx) ? 1 : -1) * (dst.x - nextx);  
        stepnum += ((dst.y >= nexty) ? 1 : -1) * (dst.y - nexty);  
      
        return stepnum * 10;  
    }  
      
    Node * IsInCloseMap(int i, int j)  
    {  
        int pos = 0;  
        for (vector<Node>::const_iterator it = close_map.begin(); it != close_map.end(); it++, pos++)  
        {  
            if (it->x == i && it->y == j )  
            {  
                return &close_map[pos];  
            }  
        }  
      
        return NULL;  
    }  
      
    Node * IsInOpenMap(int i, int j)  
    {  
        int pos = 0;  
        for (vector<Node>::const_iterator it = open_map.begin(); it != open_map.end(); it++, pos++)  
        {  
            if (it->x == i && it->y == j )  
            {  
                return &(open_map[pos]);  
            }  
        }  
      
        return NULL;  
    }  
      
    void InsertCloseList(Node & node)  
    {  
        close_map.push_back(node);  
    }  
      
    void InsertOpenList(Node & node)  
    {  
        open_map.push_back(node);  
    }  
      
    /* 
     * 计算F值，返回下一步坐票 
     * 
     * param int cur: 当前计算的坐标 
     * param out next: 下一步的坐标即F值最小的一个 
     */  
    int FindNext(Node cur, Node dst, Node & next)  
    {  
        int startx = 0; /* 遍历的横向坐标起点 */  
        int starty = 0; /* 遍历的纵向坐标起点 */  
        int endx = 0;   /* 遍历的横向坐标终点 */  
        int endy = 0;   /* 遍历的纵向坐标终点 */  
      
        InsertCloseList(cur);  
      
        /* 计算坐标 */  
        CalcBound(cur, startx, starty, endx, endy);  
      
        /* 遍历找到F值最小的坐标 */  
        int min = 0;  
        int minx = 0;  
        int miny = 0;  
      
        Node * pnext = NULL;  
        for (int i = startx; i <= endx; i++)  
        {  
            for (int j = starty; j <= endy; j++)  
            {  
                if (IsInCloseMap(i, j))  
                {/* 已存在于关闭列表 */  
                    continue;  
                }  
      
                if (-1 == short_map[i][j])  
                {/* 不可达跳过 */  
                    continue;  
                }  
      
                /* 找到一个相邻结点，判断是否在open_list中 a*/  
                Node * node = IsInOpenMap(i, j);  
                if (NULL == node)  
                {  
                    node = new Node();  
                    node->x = i;  
                    node->y = j;  
                    node->px = cur.x;  
                    node->py = cur.y;  
                    node->g = CalcG(cur, i, j);  
                    node->h = CalcH(dst, i, j);  
                    node->f = node->g + node->h;  
      
                    InsertOpenList(*node);  
                }  
                else  
                {/* 在open_list中 */  
                    int g = CalcG(cur, i, j);  
                    if ((g + cur.g) < node->g)  
                    {  
                        node->px = cur.x;  
                        node->py = cur.y;  
                        node->g = g;  
                        node->f = node->g + node->h;  
                    }  
                }  
      
                int value = node->f;  
                if (0 == min || min > value)  
                {  
                    min = value;  
                    pnext = node;  
                }  
            }  
        }  
      
        next = * pnext;  
      
        return 0;  
    }  
      
    void Draw()  
    {  
        for (int i = 0; i < ROW; i++)  
        {  
            for (int j =0; j < COLUMN; j++)  
            {  
                printf("%2d ", short_map[i][j]);  
            }  
            printf("\n");  
        }  
        printf("\n");  
      
    }  
      
    int AStar()  
    {  
        Node Src;  
        Node Dst;  
        Node Cur;  
        Node Next;  
      
        /* 查找源和目的结点 */  
        for (int i = 0; i < ROW; i++)  
        {  
            for (int j =0; j < COLUMN; j++)  
            {  
                if (1 == short_map[i][j])  
                {  
                    Src.x = i;   
                    Src.y = j;  
                    Cur.x = i;  
                    Cur.y = j;  
                }  
                  
                if (2 == short_map[i][j])  
                {  
                    Dst.x = i;  
                    Dst.y = j;  
                }  
      
                printf("%2d ", short_map[i][j]);  
            }  
      
            printf("\n");  
        }  
        printf("\n");  
      
        /* 寻路 */  
        for (int i = 0; ; i++)  
        {  
            FindNext(Cur, Dst, Next);  
      
            if (Next.x == Dst.x && Next.y == Dst.y)  
            {  
                InsertCloseList(Next);  
                break;  
            }  
      
            Cur = Next;  
        }  
      
        /* 反向绘图 */  
        Node * node;  
        int x = Dst.x;  
        int y = Dst.y;  
          
        while(1)  
        {  
            node = IsInCloseMap(x, y);  
              
            if (NULL == node)  
            {  
                printf("[%d][%d]\n", x, y);  
                break;  
            }  
      
            if (node->x == Src.x && node->y == Src.y)  
            {  
                break;  
            }  
          
            short_map[node->px][node->py] = 1;  
            Draw();  
            x = node->px;  
            y = node->py;  
        }  
    }  
      
    int main(int argc, char ** argv)  
    {  
        AStar();  
        return 0;  
    }  
