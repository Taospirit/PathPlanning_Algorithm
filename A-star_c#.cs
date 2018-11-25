namespace PathPlanning_ACO
{
    public class APoint
    {
        public int x;
        public int y;
        public int G_value;
        public int H_value;
        public APoint father;
        public int[] UnablePoint = new int[8];

        public class AStar
        {
            public int Cost10_num = 0;
            public int Cost14_num = 0;
            public int Awidth;
            public int Aheight;
            public byte[,] Map_Inf = new byte[50, 50];
            public int[] CheckPoint = new int[8];

            List<APoint> Open_List = new List<APoint>();
            List<APoint> Close_List = new List<APoint>();

            int[] Offset_x = new int[] { -1, -1, 0, 1, 1, 1, 0, -1 };//8格偏移量数组
            int[] Offset_y = new int[] { 0, -1, -1, -1, 0, 1, 1, 1 };
            #region 初始函数
            
            public void Get_MAP()//获取地图信息
            {
                for (int i = 0; i <= Awidth; i++)
                {
                    for (int j = 0; j <= Aheight; j++)
                    {
                        if (Form_Main.Map_Color[i, j] == 1)
                        {
                            Map_Inf[i, j] = 1;//1为障碍区间
                        }
                        else
                        {
                            Map_Inf[i, j] = 0;//其他均为可达
                        }
                    }
                }
            }

            public void Refresh_MAP()//从Astar中更新Main的地图信息
            {
                Array.Copy(Map_Inf, Form_Main.Map_Color, Map_Inf.Length);
            }

            private APoint GetMinFFromOpenList()
            {
                APoint Pmin = null;
                foreach (APoint p in Open_List)
                {
                    if (Pmin == null || Pmin.G_value + Pmin.H_value > p.G_value + p.H_value)
                    {
                        Pmin = p;
                    }
                }
                return Pmin;
            }

            private bool IsInCloseList(int x, int y)
            {
                foreach (APoint p in Close_List)
                {
                    if (p.x == x && p.y == y)
                    {
                        return true;
                    }
                }
                return false;
            }

            private bool IsInOpenList(int x, int y)
            {
                foreach (APoint p in Open_List)
                {
                    if (p.x == x && p.y == y)
                    {
                        return true;
                    }
                }
                return false;
            }

            private APoint GetAPointFromCloseList(int x, int y)
            {
                foreach (APoint p in Close_List)
                {
                    if (p.x == x && p.y == y)
                    {
                        return p;
                    }
                }
                return null;
            }

            private APoint GetAPointFromOpenList(int x, int y)
            {
                foreach (APoint p in Open_List)
                {
                    if (p.x == x && p.y == y)
                    {
                        return p;
                    }
                }
                return null;
            }

            private int Get_CostG(APoint p)
            {
                if (p.father == null)//根节点G值为0
                {
                    return 0;
                }
                if (p.x == p.father.x || p.y == p.father.y)//横向、纵向移动
                {
                    return p.father.G_value + 10;
                }
                else//斜方向移动
                {
                    return p.father.G_value + 14;
                }
            }

            private int Get_CostH(APoint p, APoint P_end)
            {
                return Math.Abs(p.x - P_end.x) + Math.Abs(p.y - P_end.y);//计算曼哈顿距离
            }
            #endregion

            #region 检查周围节点并更新G值，导入OpenList中
            private void Check8Point(APoint P_now, byte[,] Map, ref APoint P_end)
            {
                for(int i = 0; i <= 7; i++)
                {
                    int X_temp = P_now.x + Offset_x[i];
                    int Y_temp = P_now.y + Offset_y[i];
                    if (X_temp >= 0 && X_temp <= Awidth && Y_temp >= 0 && Y_temp <= Aheight)
                    {
                        if (Map[X_temp, P_now.y] == 1 || Map[P_now.x, Y_temp] == 1)//若po到pt的反对角上有障碍
                        {
                            continue;//忽略当前方格
                        }
                        if (Map[X_temp, Y_temp] == 0 && !IsInCloseList(X_temp, Y_temp))//0为可行区间
                        {
                            if (IsInOpenList(X_temp, Y_temp))//如果P_next在OpenList中且不在UnableList中
                            {
                                APoint P_next = GetAPointFromOpenList(X_temp, Y_temp);
                                int G_new = 0;
                                if (P_now.x == P_next.x || P_now.y == P_next.y)
                                {
                                    G_new = P_now.G_value + 10;
                                }
                                else
                                {
                                    G_new = P_now.G_value + 14;
                                }
                                if (G_new < P_next.G_value)//更新P_next的G值
                                {
                                    Open_List.Remove(P_next);
                                    P_next.father = P_now;
                                    P_next.G_value = G_new;
                                    Open_List.Add(P_next);
                                }
                            }
                            else
                            {//不在开启列表中
                                APoint P_next = new APoint();
                                P_next.x = X_temp;
                                P_next.y = Y_temp;
                                P_next.father = P_now;
                                P_next.G_value = Get_CostG(P_next);
                                P_next.H_value = Get_CostH(P_next, P_end);
                                Open_List.Add(P_next);
                            }
                        }
                    }
                }
            }

            private void Check8Point_PASS(APoint P_now, byte[,] Map, ref APoint P_end)
            {
                for (int i = 0; i <= 7; i++)
                {
                    int X_temp = P_now.x + Offset_x[i];
                    int Y_temp = P_now.y + Offset_y[i];
                    if (X_temp >= 0 && X_temp <= Awidth && Y_temp >= 0 && Y_temp <= Aheight)
                    {
                        if (Map[X_temp, Y_temp] == 0 && !IsInCloseList(X_temp, Y_temp))//0为可行区间
                        {
                            if (IsInOpenList(X_temp, Y_temp))//如果P_next在OpenList中且不在UnableList中
                            {
                                APoint P_next = GetAPointFromOpenList(X_temp, Y_temp);
                                int G_new = 0;
                                if (P_now.x == P_next.x || P_now.y == P_next.y)
                                {
                                    G_new = P_now.G_value + 10;
                                }
                                else
                                {
                                    G_new = P_now.G_value + 14;
                                }
                                if (G_new < P_next.G_value)//更新P_next的G值
                                {
                                    Open_List.Remove(P_next);
                                    P_next.father = P_now;
                                    P_next.G_value = G_new;
                                    Open_List.Add(P_next);
                                }
                            }
                            else
                            {//不在开启列表中
                                APoint P_next = new APoint();
                                P_next.x = X_temp;
                                P_next.y = Y_temp;
                                P_next.father = P_now;
                                P_next.G_value = Get_CostG(P_next);
                                P_next.H_value = Get_CostH(P_next, P_end);
                                Open_List.Add(P_next);
                            }
                        }
                    }
                }
            }
            #endregion

            #region 两个寻路函数
            public void GetWay(APoint P_start, APoint P_end)//P_start、P_end两点参数
            {
                Get_MAP();//获取地图信息
                Open_List.Add(P_start);//首个添加P_start
                while (!(IsInOpenList(P_end.x, P_end.y) || Open_List.Count == 0))
                {
                    APoint P_now = GetMinFFromOpenList();
                    Open_List.Remove(P_now);
                    Close_List.Add(P_now);
                    Check8Point(P_now, Map_Inf, ref P_end);
                }
                APoint P = GetAPointFromOpenList(P_end.x, P_end.y);
                if (P == null)
                {
                    MessageBox.Show("无可行路径！");
                }
                else
                {
                    while (P.father != null)
                    {
                        if (P.x == P.father.x || P.y == P.father.y)
                        {
                            Cost10_num++;
                        }
                        else
                        {
                            Cost14_num++;
                        }
                        P = P.father;
                        Map_Inf[P.x, P.y] = 4;//标记上色
                    }
                }
                Refresh_MAP();
            }

            public void GetWay_PASS(APoint P_start, APoint P_end)//P_start、P_end两点参数
            {
                Get_MAP();//获取地图信息
                Open_List.Add(P_start);//首个添加P_start
                while (!(IsInOpenList(P_end.x, P_end.y) || Open_List.Count == 0))
                {
                    APoint P_now = GetMinFFromOpenList();
                    Open_List.Remove(P_now);
                    Close_List.Add(P_now);
                    Check8Point_PASS(P_now, Map_Inf, ref P_end);
                }
                APoint p = GetAPointFromOpenList(P_end.x, P_end.y);
                if (p == null)
                {
                    MessageBox.Show("无可行路径！");
                }
                else
                {
                    while (p.father != null)
                    {
                        if (p.x == p.father.x || p.y == p.father.y)
                        {
                            Cost10_num++;
                        }
                        else
                        {
                            Cost14_num++;
                        }
                        p = p.father;
                        Map_Inf[p.x, p.y] = 4;//标记上色
                    }
                }
                Refresh_MAP();
            }
            #endregion

        }
    }
}
