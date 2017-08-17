# 自动寻路

## 1.自动寻路的背景

&emsp; &emsp; 游戏中，当玩家需要寻找某个NPC或者前往指定坐标时，往往需要自动寻路，开启自动寻路后角色会按照最优路径自动行走，优化了玩家的体验。

&emsp; &emsp; 抽象一点，给定两个点，一个起点，一个终点，和一个表示地图的数据(关于地图表示，有非常多的情况，此处以基本的网格为例)。地图的数据上可能存在一些无法到达的点(类似游戏中的河流，建筑等不可达区域)，现在要求给出起始点到终点的最短路径。

&emsp; &emsp; 与Dijstra算法相比较，Dijstra感觉更像是毫无方向的四周延伸搜索。而A*算法增加了一个距离的估值。


## 2.A*自动寻路

&emsp; &emsp; A*寻路算法的核心在于一个距离估算公式：
F = G + H

&emsp; &emsp; G表示从上一步走到的点到当前点的距离，此距离通常为固定值，因为相邻两点之间只有两种关系：一是与当前点直接相邻，即当前点的上下左右邻接点，二是与当前点斜相邻：左上、右上、左下、右下。前者距离为1，后者距离为1.4，此处为避免浮点数运算，直接扩大10倍为10和14.

&emsp; &emsp; H表示当前点到目标点的距离估值，它计算从当前格到目的格之间水平和垂直的方格的数量总和，忽略对角线方向，也被称作曼哈顿距离。很重要的一点，我们忽略了一切障碍物。这是对剩余距离的一个估算，而非实际值，这也是这一方法被称为启发式的原因。

&emsp; &emsp; 除此之外，我们还需要两个列表，一个OpenList，一个CloseList。前者代表当前待检查的方格，后者表示所有不再需要检查的方格。


## 3.开始搜索

搜索步骤如下：

1. 将起始点加入OpenList

2. 重复以下过程：

3. 如果OpenList为空，则没有路径。否则继续以下步骤：
 
4. 从OpenList中找到F值最小的元素，作为当前步方块，并从OpenList中去除该方块，将该方块放入CloseList中

5. 如果该F值最小的元素为目标点，则找到路径，跳出循环。否则继续以下步骤：

5. 遍历当前选中的方块的8个方块，如果被遍历的方块不可行走或者在ColseList中则跳过步骤4和步骤5，否则进行步骤4和步骤5：

6. 更新被遍历方块的G值，如果原G值大于以当前步方块为父方块后的G值 （找到了更好的路径），则更新G值和F值，并以当前步方块为父方块。

7. 将被遍历的方块加入OpenList，如果已经在OpenList则不再重复这一步。

## 4.代码

<pre><code>

const int MAX_WIDTH = 30;
const int MAX_HEIGHT = 30;

const int OBLIQUE_DIS = 14;
const int STRAIGHT_DIS = 10;

const int MAX = 999999;

const int START_X = 1;
const int START_Y = 1;

const int END_X = 22;
const int END_Y = 22;

const int INDEX_X[3] = {-1, 0, 1};
const int INDEX_Y[3] = {-1, 0, 1};

struct Node
{
public:
	Node() : nF(MAX * 2), nG(MAX), nH(MAX), nPosX(0), nPosY(0), bAvailable(true), bInOpenList(false), pParent(NULL) {}
	int nF;
	int nG;
	int nH;
	int nPosX;
	int nPosY;
	bool bAvailable;
	bool bInOpenList;
	Node *pParent;
};

Node Map[MAX_HEIGHT + 1][MAX_WIDTH + 1];

//vector<Node*> vecOpenList;
multimap<int, Node*> mapOpenList;
list<Node*> vecCloseList;

void InitMap(int nStartRow, int nStartCol, int nEndRow, int nEndCol)
{
	for (int i = 0; i <= MAX_HEIGHT; i++)
	{
		for (int j = 0; j <= MAX_WIDTH; j++)
		{
			// set coordinate
			Map[i][j].nPosX = j;
			Map[i][j].nPosY = i;

			// set manhattan distance
			Map[i][j].nH = 10 * (abs(nEndRow - j) + abs(nEndCol - i));
		}
	}

	// set start_point
	Map[nStartRow][nStartCol].nF = 0;
	Map[nStartRow][nStartCol].nG = 0;
	Map[nStartRow][nStartCol].nH = 0;
}

void ModifyDistance(int nCurNodeX, int nCurNodeY, int nTargetNodeX, int nTargetNodeY)
{
	int nDifferX = abs(nCurNodeX - nTargetNodeX);
	int nDifferY = abs(nCurNodeY - nTargetNodeY);
	if (0 == (nDifferX + nDifferY))
		return;

	int nObliqueDis = Map[nCurNodeY][nCurNodeX].nG + OBLIQUE_DIS;
	int nStraightDis = Map[nCurNodeY][nCurNodeX].nG + STRAIGHT_DIS;

	if (2 == (nDifferX + nDifferY))
	{
		if (Map[nTargetNodeY][nTargetNodeX].nG > nObliqueDis)
		{
			Map[nTargetNodeY][nTargetNodeX].nG = nObliqueDis;
			Map[nTargetNodeY][nTargetNodeX].nF = Map[nTargetNodeY][nTargetNodeX].nG + Map[nTargetNodeY][nTargetNodeX].nH;
			Map[nTargetNodeY][nTargetNodeX].pParent = &Map[nCurNodeY][nCurNodeX];
		}
	}
	else
	{
		if (Map[nTargetNodeX][nTargetNodeY].nG > nStraightDis)
		{
			Map[nTargetNodeY][nTargetNodeX].nG = nStraightDis;
			Map[nTargetNodeY][nTargetNodeX].nF = Map[nTargetNodeY][nTargetNodeX].nG + Map[nTargetNodeY][nTargetNodeX].nH;
			Map[nTargetNodeY][nTargetNodeX].pParent = &Map[nCurNodeY][nCurNodeX];
		}
	}
}

bool compare(Node *a, Node *b)
{
	return a->nF > b->nF;
}

int main(void)
{
	clock_t start, finish;

	start = clock();

	InitMap(START_X, START_Y, END_X, END_Y);

	Map[START_Y][START_X].bInOpenList = true;
	//vecOpenList.push_back(&Map[START_Y][START_X]);
	mapOpenList.insert(make_pair(Map[START_Y][START_X].nF, &Map[START_Y][START_X]));
	Map[3][3].bAvailable = false;

	multimap<int, Node*>::iterator itOpenList;
	while (1)
	{
		//sort(vecOpenList.begin(), vecOpenList.end(), compare);

		//int nTailIndex = vecOpenList.size() - 1;
		//int nCurNodeX = vecOpenList[nTailIndex]->nPosX;
		//int nCurNodeY = vecOpenList[nTailIndex]->nPosY;

		itOpenList = mapOpenList.begin();
		int nCurNodeX = itOpenList->second->nPosX;
		int nCurNodeY = itOpenList->second->nPosY;

		if (nCurNodeX == END_X && nCurNodeY == END_Y)
			break;

		//vecOpenList.pop_back();
		mapOpenList.erase(itOpenList);

		Map[nCurNodeY][nCurNodeX].bInOpenList = false;
		Map[nCurNodeY][nCurNodeX].bAvailable = false;
		vecCloseList.push_back(&Map[nCurNodeY][nCurNodeX]);

		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				int nTargetNodeX = nCurNodeX + INDEX_X[i];
				int nTargetNodeY = nCurNodeY + INDEX_Y[j];
				if (Map[nTargetNodeY][nTargetNodeX].bAvailable)
				{
					ModifyDistance(nCurNodeX, nCurNodeY, nTargetNodeX, nTargetNodeY);
					if ( !Map[nTargetNodeY][nTargetNodeX].bInOpenList)
					{
						Map[nTargetNodeY][nTargetNodeX].bInOpenList = true;
						//vecOpenList.push_back(&Map[nTargetNodeY][nTargetNodeX]);
						mapOpenList.insert(make_pair(Map[nTargetNodeY][nTargetNodeX].nF, &Map[nTargetNodeY][nTargetNodeX]));
					}
				}
			}
		}
	}

	finish = clock();

	cout << "耗时：" << finish - start << endl;

	itOpenList = mapOpenList.begin();
	int nCurNodeX = itOpenList->second->nPosX;
	int nCurNodeY = itOpenList->second->nPosY;

	//int nTailIndex = vecOpenList.size() - 1;
	//int nCurNodeX = vecOpenList[nTailIndex]->nPosX;
	//int nCurNodeY = vecOpenList[nTailIndex]->nPosY;
	Node *pIndex = &Map[nCurNodeY][nCurNodeX];
	while (pIndex->pParent != NULL)
	{
		cout << "X: " << pIndex->nPosX << "      Y: " << pIndex->nPosY << endl;
		pIndex = pIndex->pParent;
	}
	cout << "X: " << pIndex->nPosX << "      Y: " << pIndex->nPosY << endl;

	system("PAUSE");
	return 0;
}
</code></pre>

## 4.优化

A*算法中关于OpenList的操作包含了频繁的插入和删除元素，排序，以及对最小F值的访问。对此我们可以着手思考优化。以stl中的常用容器为例：

1.vector支持随机访问，但插入和删除会引发容器的内存增长导致相率降低

2.multiset实现本质是红黑树，插入和删除会引发树的反转，但相比vector整个容器的拷贝开销小得多。因为实际已经按照关键字排序过，所以对最小值的访问也不会影响效率。由于multiset元素按照本情况需要包含坐标点，F，G，H值等较多数据，为自定义数据，需要手动指定一个排序函数，较为麻烦。

3.list本质为链表，插入和删除开销极低，但问题在于需要先找到插入和开销的点，无法使用二分法，只能挨着顺序查找，故效率不高。

4.multimap本质也是红黑树，但储存了键值对，各方面效率与multiset类似。好处在于可以用F值做key，节点信息做value，由于F值为int非自定义类型，故不需要手动指定排序函数，更加方便。

综上，此处采用multimap为OpenList的数据结构，采用list作为CloseList的数据结构。

## 5.思考

该版本只是A*算法的一个初步了解，应结合具体情况具体分析。如果在3D游戏中，很可能需要结合空间八叉树来修改算法。又或者，地图的表示就有非常多种情况，如果不是正方形分割而是多边形，此时又需要多些考虑。

此外，此算法适用于静态寻路，但如果方格的权重是动态变化的，那么此时需要用到A* 算法的动态改进即D*寻路算法，具体细节不在此详述。

