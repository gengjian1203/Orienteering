///////////////////////////////////////////////////////////////////
// 文	件：orienteering.cpp
// 功	能：将example1.txt文件中信息读取出来，‘#’为障碍物，‘.’为可行
//			区域，经过运算找到一个起点为‘S’，终点为‘G’，且经过所有
//			‘@’点的最短路径
// 思	想：将问题分解。首先通过A*算法计算出任意两点之间的最短路径和步骤
//			，接下来就是解决哈密顿路径问题。最后通过Floyd算法得出满足题
//			目要求的最短路径。
//	A*算法：fAstar = gAstar + hAstar
//			fAstar = 起点经过节点到终点的估价值
//			gAstar = 起点到达节点的实际代价值
//			hAstar = 节点到达终点的最佳路径的估价值
//			目标为在节点到达终点时，fAstar数值最小，即为最短路径所用步数
//			（为简化运算，本题中hAstar的取值为两点之间的直线距离的平方）
// 待解	决：节点数最多计算15个点，无法达到题目要求的20个点
// 作	者：gengjian1203
// 邮	箱：gengjian1203@foxmail.com
// 日	期：2014.10.20
///////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <errno.h>
#include <string.h>

// 根据题干，设定以下宏
// 1 <= width <= 100   1 <= height <= 100
#define MAX_WIDTH 100
#define MAX_HEIGHT 100
// The maximum number of point is 20 (start + goal + check)
// 15个点为内存极限 -.-!~~如在内存可以加大的情况下可将MAX_POINT宏的数加大
// 但占用内存成几何倍数增长）
#define MAX_POINT 15
#define MAX_CHECK MAX_POINT - 2

struct MyPoint
{
    // 该点属性
    int x;
    int y;

};

struct MyNode
{
    // 该点属性
    MyPoint point;
    // 该点star参数
    int fAstar;
    int hAstar;
    int gAstar;
    // 指向上一步骤行走的节点
    MyNode* pParent;
    // 指向链表中下个元素
    MyNode* pNext;

};

class Orienteering
{
public:
    void main();

private:
    // 棋盘信息-宽
    int m_nWidth;
    // 棋盘信息-高
    int m_nHeight;
    // 棋盘信息-check点个数
    int m_nCheckCount;
    // 棋盘信息-明细
    char m_chess[MAX_HEIGHT][MAX_WIDTH];
    // 棋盘信息-节点信息
    MyPoint m_point[MAX_POINT];					// 第0元素为start，第1元素为goal，接下来一次为check point。
    // 哈密顿路径-距离矩阵
    int m_nHamilRect[MAX_POINT][MAX_POINT];
    // 哈密顿路径-行走最短步数
    int m_nDist[(1<<MAX_POINT)][MAX_POINT];		// 问题：内存申请可能过大！！！！！

    MyNode* listOPEN;
    MyNode* listCLOSED;

private:
    ///////////////////////////////////////////////////////////////////
    // 分析可获知信息准备阶段
    ///////////////////////////////////////////////////////////////////
    // 输出调试信息
    void showDebug();
    // 创建棋盘信息数组
    bool createChess();
    // 分析棋盘
    bool analyseChess();
    ///////////////////////////////////////////////////////////////////
    // A*算法操作阶段
    ///////////////////////////////////////////////////////////////////
    // 获取节点N的hAstar值
    int gethAstar(MyPoint pointN, MyPoint pointG);
    // 获取listOPEN表中hAstar最小的节点
    MyNode* getBestNode();
    // 将该点从listOPEN表移动到listCLOSED表中
    void moveOPENtoCLOSED(MyPoint point);
    // 将该点从listCLOSED表移动到listOPEN表中
    void moveCLOSEDtoOPEN(MyPoint point);
    // 判断该点是否为可行区域
    bool isSureNode(MyPoint point);
    // 判断该节点是否在listOPEN表中
    MyNode* isNodeInlistOPEN(MyPoint point);
    // 判断该节点是否在listCLOSED表中
    MyNode* isNodeInlistCLOSED(MyPoint point);
    // 将该节点添加到listOPEN表中
    void inserttolistOPEN(MyNode* pInsertNode);
    // 生成的后续节点情况处理
    void generateList(MyNode* pParentNode, MyPoint pointChild, MyPoint pointGoal);
    // 生成最佳节点下的子节点，填入listOPEN
    void createChildNodeIntoOpen(MyNode *pParentNode, MyPoint pointGoal);
    // 销毁表指定表
    void DestroyList(MyNode *pHead);
    // A*算法主要流程
    int AstarMain(MyPoint pointGoal, MyPoint pointStart);
    ///////////////////////////////////////////////////////////////////
    // 求最短哈密顿路径阶段
    ///////////////////////////////////////////////////////////////////
    // 生成哈密顿路径距离矩阵m_nHamilRect
    bool createHamilRect();
    // 运算哈密顿Floyd
    void hamiltonianFloyd();
    // 求哈密顿最短路径
    int hamiltonianPath(int nIndexS, int nIndexG);
};

///////////////////////////////////////////////////////////////////
// 函	数：showDebug
// 作	用：输出调试信息
// 参	数：void
// 返	回：void
///////////////////////////////////////////////////////////////////
void Orienteering::showDebug()
{
    printf("\n");
    printf("W:%d, H:%d\n", m_nWidth, m_nHeight);
    for (int i = 0; i < m_nHeight; i++)
    {
        for (int j = 0; j < m_nWidth; j++)
        {
            printf("%c", m_chess[i][j]);
        }
        printf("\n");
    }
    printf("Start Point:(%d, %d)\n", m_point[0].x, m_point[0].y);
    printf("Goal Point:(%d, %d)\n", m_point[1].x, m_point[1].y);
    for (int k = 2; k < m_nCheckCount; k++)
    {
        printf("Check Point%d:(%d, %d)\n", k, m_point[k].x, m_point[k].y);
    }

    for (int i = 0; i < m_nCheckCount; i++)
    {
        for (int j = 0; j < m_nCheckCount; j++)
        {
            printf("%d|", m_nHamilRect[i][j]);
        }
        printf("\n");
    }
}

///////////////////////////////////////////////////////////////////
// 函	数：createChess
// 作	用：创建棋盘信息数组
// 参	数：void
// 返	回：bool		返回创建的棋盘信息是否合法
///////////////////////////////////////////////////////////////////
bool Orienteering::createChess()
{
    FILE* fp = 0x00;
    char ln[MAX_WIDTH];
    fp = fopen("example1.txt", "r");
    if (0x00 == fp)
    {
        printf("Can not open file.\n");
        return false;
    }
    // 读取地图行列数
    if (NULL == fgets(ln, MAX_WIDTH, fp))
    {
        printf("Get width and height error.\n");
        return false;
    }
    if (-1 == sscanf(ln, "%d,%d", &m_nWidth, &m_nHeight))
    {
        printf("The width or height is error.%d,%s.\n", errno, strerror(errno));
        return false;
    }
    if ((1 > m_nWidth) || (MAX_WIDTH < m_nWidth))
    {
        printf("width is error.(1 <= width <= 100)\n");
        return false;
    }
    if ((1 > m_nHeight) || (MAX_HEIGHT < m_nHeight))
    {
        printf("height is error.(1 <= height <= 100)\n");
        return false;
    }
    for (int i = 0; i < m_nHeight; i++)
    {
        if (NULL == fgets(m_chess[i], MAX_WIDTH, fp))
        {
            printf("Get map data error.\n");
            return false;
        }
    }
    return true;
//    scanf("%d %d\n", &m_nWidth, &m_nHeight);
//    if ((1 > m_nWidth) || (100 < m_nWidth))
//    {
//        printf("width is error.(1 <= width <= 100)\n");
//        return false;
//    }
//    if ((1 > m_nHeight) || (100 < m_nHeight))
//    {
//        printf("height is error.(1 <= height <= 100)\n");
//        return false;
//    }
//    for (int i = 0; i < m_nHeight; i++)
//    {
//        gets(m_chess[i]);
//    }
//    return true;
}

///////////////////////////////////////////////////////////////////
// 函	数：analyseChess
// 作	用：分析棋盘
// 参	数：void
// 返	回：bool		分析输入数据的安全性，通过为真，否则为假
///////////////////////////////////////////////////////////////////
bool Orienteering::analyseChess()
{
    int nMaxCheck = MAX_CHECK;
    bool bStart = false;
    bool bGoal = false;

    m_nCheckCount = 2;

    for (int i = 0; i < m_nHeight; i++)
    {
        for (int j = 0; j < m_nWidth; j++)
        {
            if ('S' == m_chess[i][j])
            {
                if (!bStart)
                {
                    m_point[0].x = j;
                    m_point[0].y = i;
                    bStart = true;
                }
                else
                {
                    printf("'S' has exceeded the maximum limit.(max:1)\n");
                    return false;
                }
            }
            else if ('G' == m_chess[i][j])
            {
                if (!bGoal)
                {
                    m_point[1].x = j;
                    m_point[1].y = i;
                    bGoal = true;
                }
                else
                {
                    printf("'G' has exceeded the maximum limit.(max:1)\n");
                    return false;
                }

            }
            else if ('@' == m_chess[i][j])
            {
                if (m_nCheckCount < nMaxCheck)
                {
                    m_point[m_nCheckCount].x = j;
                    m_point[m_nCheckCount].y = i;
                    m_nCheckCount++;
                }
                else
                {
                    printf("'@' has exceeded the maximum limit.(max:%d)\n", MAX_CHECK);
                    return false;
                }
            }
            else
            {
                if (!(('.' == m_chess[i][j]) || ('#' == m_chess[i][j])))
                {
                    // 如果遇到非以上字符，则返回错误
                    printf("The discovery of illegal characters.\n");
                    return false;
                }
            }
        }
    }
    if (!bStart)
    {
        printf("Did not find 'S'.\n");
        return false;
    }
    if (!bGoal)
    {
        printf("Did not find 'G'.\n");
        return false;
    }
    return true;
}

///////////////////////////////////////////////////////////////////
// 函	数：gethAstar
// 作	用：获取点N的hAstar值
// 参	数：MyPoint pointN	点N的信息
// 参	数：MyPoint pointG	点G的信息
// 返	回：int			返回pointN的hAstar值
///////////////////////////////////////////////////////////////////
int Orienteering::gethAstar(MyPoint pointN, MyPoint pointG)
{
    // 尚未进行开方运算
    return ((pointG.x - pointN.x) * (pointG.x - pointN.x) + (pointG.y - pointN.y) * (pointG.y - pointN.y));
}

///////////////////////////////////////////////////////////////////
// 函	数：getBestNode
// 作	用：获取listOPEN表中hAstar最小的节点
// 参	数：void
// 返	回：MyNode*		返回指向listOPEN表中hAstar值最小的节点指针
///////////////////////////////////////////////////////////////////
MyNode* Orienteering::getBestNode()
{
    if (NULL != listOPEN)
    {
        // 操作指针
        MyNode* pNode = listOPEN;
        // 最佳结果指针
        MyNode* pBestNode = listOPEN;
        // 最佳fAstar
        int nBestfAstar = listOPEN->fAstar;

        while( pNode )
        {
            if ( pNode->fAstar < nBestfAstar )
            {
                pBestNode = pNode;
                nBestfAstar = pNode->fAstar;
            }
            pNode = pNode->pNext;
        }

        // 将Astartf最小的一个元素返回
        return pBestNode;
    }
    else
    {
        return NULL;
    }
}

///////////////////////////////////////////////////////////////////
// 函	数：moveOPENtoCLOSED
// 作	用：将该点从listOPEN表移动到listCLOSED表中
// 参	数：MyPoint pointN	移动点的信息
// 返	回：void
///////////////////////////////////////////////////////////////////
void Orienteering::moveOPENtoCLOSED(MyPoint point)
{
    MyNode* pOpen = listOPEN;
    MyNode* pClosed = listCLOSED;
    MyNode* pBefore = NULL;
    // 在listOPEN中找到符合要求的节点
    while ( pOpen )
    {
        if (( pOpen->point.x == point.x) && (pOpen->point.y == point.y))
        {
            break;
        }
        pBefore = pOpen;
        pOpen = pOpen->pNext;
    }
    if (!pOpen)
    {
        printf("error！在listOPEN中未找到符合要求的节点！");
        return;
    }
    // 把该节点从listOPEN表中取出
    if ( pOpen == listOPEN )
    {
        listOPEN = pOpen->pNext;
    }
    else
    {
        pBefore->pNext = pOpen->pNext;
    }
    // 找到listCLOSED表中最后
    while ( pClosed )
    {
        pBefore = pClosed;
        pClosed = pClosed->pNext;
    }
    // 将该节点添加到listCLOSED表中最后
    if ( pClosed == listCLOSED )
    {
        listCLOSED = pOpen;
    }
    else
    {
        pBefore->pNext = pOpen;
    }
    pOpen->pNext = NULL;
}

///////////////////////////////////////////////////////////////////
// 函	数：moveCLOSEDtoOPEN
// 作	用：将该点从listCLOSED表移动到listOPEN表中
// 参	数：MyPoint pointN	移动点的信息
// 返	回：void
///////////////////////////////////////////////////////////////////
void Orienteering::moveCLOSEDtoOPEN(MyPoint point)
{
    MyNode* pOpen = listOPEN;
    MyNode* pClosed = listCLOSED;
    MyNode* pBefore = NULL;
    // 在listCLOSED中找到符合要求的节点
    while (pClosed)
    {
        if ((pClosed->point.x == point.x) && (pClosed->point.y == point.y))
        {
            break;
        }
        pBefore = pClosed;
        pClosed = pClosed->pNext;
    }
    if (!pClosed)
    {
        printf("error！在listCLOSED中未找到符合要求的节点！");
        return;
    }
    // 把该节点从listCLOSED表中取出
    if (pClosed == listCLOSED)
    {
        listCLOSED = pClosed->pNext;
    }
    else
    {
        pBefore->pNext = pClosed->pNext;
    }
    // 找到listOPEN表中最后
    while (pOpen)
    {
        pBefore = pOpen;
        pOpen = pOpen->pNext;
    }
    // 将该节点添加到listOPEN表中最后
    if (pOpen == listOPEN)
    {
        listOPEN = pClosed;
    }
    else
    {
        pBefore->pNext = pClosed;
    }
    pClosed->pNext = NULL;
}

///////////////////////////////////////////////////////////////////
// 函	数：isSureNode
// 作	用：判断该点是否为可行区域
// 参	数：MyPoint pointN	判断的点的信息
// 返	回：bool	返回是否为可通行区域
///////////////////////////////////////////////////////////////////
bool Orienteering::isSureNode(MyPoint point)
{
    return ('#' != m_chess[point.y][point.x]);
}

///////////////////////////////////////////////////////////////////
// 函	数：isNodeInlistOPEN
// 作	用：判断该节点是否在listOPEN表中
// 参	数：MyPoint pointN	判断的点的信息
// 返	回：MyNode*		如果存在返回该节点指针，如果不存在返回NULL
///////////////////////////////////////////////////////////////////
MyNode* Orienteering::isNodeInlistOPEN(MyPoint point)
{
    // 操作指针
    MyNode* pNode = listOPEN;

    while( pNode )
    {
        if ( ( point.x == pNode->point.x ) && ( point.y == pNode->point.y ) )
        {
            return pNode;
        }
        pNode = pNode->pNext;
    }

    return NULL;
}

///////////////////////////////////////////////////////////////////
// 函	数：isNodeInlistCLOSED
// 作	用：判断该节点是否在listCLOSED表中
// 参	数：MyPoint pointN	判断的点的信息
// 返	回：MyNode*		如果存在返回该节点指针，如果不存在返回NULL
///////////////////////////////////////////////////////////////////
MyNode* Orienteering::isNodeInlistCLOSED(MyPoint point)
{
    // 操作指针
    MyNode* pNode = listCLOSED;

    while( pNode )
    {
        if ( ( point.x == pNode->point.x ) && ( point.y == pNode->point.y ) )
        {
            return pNode;
        }
        pNode = pNode->pNext;
    }

    return NULL;
}

///////////////////////////////////////////////////////////////////
// 函	数：inserttolistOPEN
// 作	用：将该节点添加到listOPEN表中
// 参	数：MyNode* pInsertNode	指向判断的节点的指针
// 返	回：void
///////////////////////////////////////////////////////////////////
void Orienteering::inserttolistOPEN(MyNode* pInsertNode)
{
    MyNode* pNode = listOPEN;
    MyNode* pBefore = NULL;
    // 找到listOPEN表中最后
    while (pNode)
    {
        pBefore = pNode;
        pNode = pNode->pNext;
    }
    // 将该节点添加到listOPEN表中最后
    if (pNode == listOPEN)
    {
        listOPEN = pInsertNode;
    }
    else
    {
        pBefore->pNext = pInsertNode;
    }
    pInsertNode->pNext = NULL;
}

///////////////////////////////////////////////////////////////////
// 函	数：generateList
// 作	用：生成的后续节点情况处理
// 参	数：MyNode* pParentNode	父辈节点信息
// 参	数：MyPoint pointChild	当前点信息
// 参	数：MyPoint pointGoal	目标点信息
// 返	回：void
///////////////////////////////////////////////////////////////////
void Orienteering::generateList(MyNode* pParentNode, MyPoint pointChild, MyPoint pointGoal)
{
    MyNode* pResultOPEN;
    MyNode* pResultCLOSED;
    MyNode* pNode;
    // 新开辟个节点空间
    pNode = (MyNode*)calloc(1, sizeof(MyNode));
    pNode->point.x = pointChild.x;
    pNode->point.y = pointChild.y;
    pNode->gAstar = pParentNode->gAstar + 1;
    pNode->hAstar = gethAstar(pointChild, pointGoal);
    pNode->fAstar = pNode->gAstar + pNode->hAstar;
    //pNode->pNext = NULL; // 尚未定论
    pNode->pParent = pParentNode;
    // 对该节点进行判断处理
    pResultOPEN = isNodeInlistOPEN(pointChild);
    pResultCLOSED = isNodeInlistCLOSED(pointChild);

    if (pResultOPEN)
    {
        if (pNode->fAstar < pResultOPEN->fAstar)
        {
            // 1、该节点在OPEN里，f(s)比原值小,那么就替换原先节点，做好父节点指针。
            pResultOPEN->gAstar = pNode->gAstar;
            pResultOPEN->hAstar = pNode->hAstar;
            pResultOPEN->fAstar = pNode->fAstar;
            pResultOPEN->pParent = pNode->pParent;
            // 释放空间
            free(pNode);
        }

    }
    else if (pResultCLOSED)
    {
        if (pNode->fAstar < pResultCLOSED->fAstar)
        {
            // 2、该节点在CLOSE里，f(s)比原值小，将CLOSE放入OPEN中，做好父节点指针。
            pResultCLOSED->gAstar = pNode->gAstar;
            pResultCLOSED->hAstar = pNode->hAstar;
            pResultCLOSED->fAstar = pNode->fAstar;
            pResultCLOSED->pParent = pNode->pParent;
            moveCLOSEDtoOPEN(pointChild);   // 算法重复有空改一下
            // 释放空间
            free(pNode);
        }
    }
    else
    {
        // 3、该节点不在OPEN、CLOSED里，那么就加到OPEN表中，做好父节点指针。
        inserttolistOPEN(pNode);
    }

}

///////////////////////////////////////////////////////////////////
// 函	数：createChildNodeIntoOpen
// 作	用：最佳节点下的所有后续节点，并加入listOPEN表
// 参	数：MyNode* pParentNode	父辈节点信息
// 参	数：MyPoint pointGoal	目标点信息
// 返	回：void
///////////////////////////////////////////////////////////////////
void Orienteering::createChildNodeIntoOpen(MyNode *pParentNode, MyPoint pointGoal)
{
    MyPoint pointChild;
    // 上
    pointChild.x = pParentNode->point.x;
    pointChild.y = pParentNode->point.y - 1;
    if ( pointChild.y >= 0 )
    {
        if ( isSureNode( pointChild ) )
        {
            generateList(pParentNode, pointChild, pointGoal);
        }
    }
    // 下
    pointChild.x = pParentNode->point.x;
    pointChild.y = pParentNode->point.y + 1;
    if ( pointChild.y < m_nHeight )
    {
        if ( isSureNode( pointChild ) )
        {
            generateList(pParentNode, pointChild, pointGoal);
        }
    }
    // 左
    pointChild.x = pParentNode->point.x - 1;
    pointChild.y = pParentNode->point.y;
    if ( pointChild.x >= 0 )
    {
        if ( isSureNode( pointChild ) )
        {
            generateList(pParentNode, pointChild, pointGoal);
        }
    }
    // 右
    pointChild.x = pParentNode->point.x + 1;
    pointChild.y = pParentNode->point.y;
    if ( pointChild.y < m_nWidth )
    {
        if ( isSureNode( pointChild ) )
        {
            generateList(pParentNode, pointChild, pointGoal);
        }
    }
    return;
}

///////////////////////////////////////////////////////////////////
// 函	数：DestroyList
// 作	用：销毁表指定表
// 参	数：MyNode *pHead		需要销毁的链表头指针
// 返	回：void
///////////////////////////////////////////////////////////////////
void Orienteering::DestroyList(MyNode *pHead)
{
    if (NULL != pHead)
    {
        MyNode *p;
        while (pHead)
        {
            p = pHead->pNext;
            free(pHead);
            pHead = p;
        }
    }
}

///////////////////////////////////////////////////////////////////
// 函	数：AstarMain
// 作	用：A*算法主要流程
// 参	数：MyPoint pointGoal		起点信息（A*算法结果是逆向输出）
// 参	数：MyPoint pointStart		终点信息
// 返	回：void
///////////////////////////////////////////////////////////////////
int Orienteering::AstarMain(MyPoint pointGoal, MyPoint pointStart)
{
    MyNode* pNode;
    MyNode* pNodeBest;
    // 生成listOPEN表和listCLOSED表
    listOPEN = NULL;
    listCLOSED = NULL;
    // 路径相关
    int nPathCount = -1;
    MyNode* pPath = NULL;

    // 生成起始节点
    pNode = (MyNode*)calloc( 1, sizeof( MyNode ) );
    // 录入起始点坐标等属性信息
    pNode->point.x = pointStart.x;
    pNode->point.y = pointStart.y;
    pNode->gAstar = 0;
    pNode->hAstar = gethAstar(pNode->point, pointGoal);
    pNode->fAstar = pNode->gAstar + pNode->hAstar;
    pNode->pParent = NULL;
    pNode->pNext = NULL;
    // 将起始点放入listOPEN表中
    listOPEN = pNode;

    while( true )
    {
        // 从listOPEN表中取得一个fAstar值最好（小）的节点
        pNodeBest = getBestNode();
        if (pNodeBest)
        {
            // 将最佳点从OPEN放入CLOSED里
            moveOPENtoCLOSED(pNodeBest->point);

            if ((pNodeBest->point.x == pointGoal.x) && (pNodeBest->point.y == pointGoal.y))
            {
                // 如果该节点是目标节点就退出
                break;
            }
            else
            {
                // 如果该节点不是目标节点就使其生成子节点，添加到listOPEN表中
                createChildNodeIntoOpen(pNodeBest, pointGoal);
            }
        }
        else
        {
            //listOPEN表无点可取，该题无解，寻路失败
            break;
        }

    }
    //////////////////////////////////////////////////////////////////////////
    // 得到路径
    pPath = pNodeBest;

    while (pPath)
    {
        //printf("(%d, %d)=>", pPath->point.x, pPath->point.y);
        pPath = pPath->pParent;
        nPathCount++;
    }
    //printf("\nResult moves %d times.\n", nPathCount);

    // 释放OPEN和CLOSED表内存
    DestroyList(listOPEN);
    DestroyList(listCLOSED);

    return nPathCount;

}

///////////////////////////////////////////////////////////////////
// 函	数：createHamilRect
// 作	用：生成距离矩阵
// 参	数：void
// 返	回：bool		如果有任意两点不通返回假，任意两点都通都通返回真
///////////////////////////////////////////////////////////////////
bool Orienteering::createHamilRect()
{
    int nPath;

    for (int i = 0; i < m_nCheckCount; i++)
    {
        for (int j = i; j < m_nCheckCount; j++)
        {
            if (i == j)
            {
                m_nHamilRect[i][j] = 0;
            }
            else
            {
                nPath = AstarMain(m_point[i], m_point[j]);
                // 如果发现不通路，直接返回为假
                if (-1 == nPath)
                {
                    return false;
                }
                m_nHamilRect[i][j] = nPath;
                m_nHamilRect[j][i] = nPath;
            }
        }
    }
    return true;
}

///////////////////////////////////////////////////////////////////
// 函	数：hamiltonianFloyd
// 作	用：运算哈密顿floyd
// 参	数：void
// 返	回：void
///////////////////////////////////////////////////////////////////
void Orienteering::hamiltonianFloyd()
{
    for(int k = 0; k < m_nCheckCount; k++)
    {
        for(int i = 0; i < m_nCheckCount; i++)
        {
            for(int j = 0; j < m_nCheckCount; j++)
            {
                if(m_nHamilRect[i][k] + m_nHamilRect[k][j] < m_nHamilRect[i][j])
                {
                    m_nHamilRect[i][j] = m_nHamilRect[i][k] + m_nHamilRect[k][j];
                }
            }
        }
    }
}

///////////////////////////////////////////////////////////////////
// 函	数：hamiltonianPath
// 作	用：求哈密顿最短路径
// 参	数：int nIndexS		起点的数组索引值
// 参	数：int nIndexG		终点的数组索引值
// 返	回：int		返回设置起始两点，且经过所有节点最短路径的步数
///////////////////////////////////////////////////////////////////
int Orienteering::hamiltonianPath(int nIndexS, int nIndexG)
{
    int pow2[MAX_POINT];
    for(int o = 0; o <= m_nCheckCount; o++)
    {
        pow2[o] = (1<<o);
    }
    for(int m = 0; m < pow2[m_nCheckCount]; m++)
    {
        for(int n = 0; n < m_nCheckCount; n++)
        {
            m_nDist[m][n] = INT_MAX;
        }
    }
    hamiltonianFloyd();
    m_nDist[pow2[nIndexS]][nIndexS] = 0;

    for(int i = 0; i < pow2[m_nCheckCount]; i++)
    {
        for(int j = 0; j < m_nCheckCount; j++)
        {
            if((i & pow2[j]) && m_nDist[i][j] < INT_MAX)
            {
                for(int k = 0; k < m_nCheckCount; k++)
                {
                    if(!(i & pow2[k]))
                    {
                        if(m_nDist[(i|pow2[k])][k] > m_nDist[i][j] + m_nHamilRect[j][k])
                        {
                            m_nDist[(i|pow2[k])][k] = m_nDist[i][j] + m_nHamilRect[j][k];
                        }
                    }
                }
            }
        }
    }
    return m_nDist[pow2[m_nCheckCount]-1][nIndexG];
}

///////////////////////////////////////////////////////////////////
// 函	数：main
// 作	用：Orienteering类的入口主函数
// 参	数：void
// 返	回：void
///////////////////////////////////////////////////////////////////
void Orienteering::main()
{
    // 通过example1.txt中的信息存放到m_chess数组中
    if (!createChess())
    {
        // 数据非法则直接返回
        return;
    }
    // 分析m_chess数组，提取出解题所需信息
    if (!analyseChess())
    {
        // 数据非法则直接返回
        return;
    }
    // 根据A*算法，算出任意两点之间的最短距离，将数据存放到m_nHamiRect数组中
    // 构成解决哈密顿最短路径的权重矩阵
    if (createHamilRect())
    {
        // 如果发现点点相通，再进行最短路径运算
        showDebug();
        printf("So.The result of steps away:%d\n", hamiltonianPath(0, 1));

    }
    else
    {
        // 如果发现只要存在两点无法相通，则该题无解，返回-1
        //showDebug();
        printf("-1\n");

    }
    return ;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
// 函	数：main
// 作	用：程序主函数
// 参	数：int argc		输入参数个数
// 参	数：char* argv[]	命令行参数
// 返	回：int		给操作系统返回执行情况
///////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
    Orienteering o;
    o.main();
    //////////////////////////////////////////////////////////////////////
    // 调试
    //system("pause");
    return 0;
}
