
class PriorityQueue1
{
    struct PriorityQueueElement
    {
        public int id;
        public float priority;
    }
    PriorityQueueElement[] heap;
    bool[] hash;
    int size = -1;
    int maxSize;

    public PriorityQueue1(int size)
    {
        maxSize = size;
        heap = new PriorityQueueElement[size];
        hash = new bool[size];
        Clear();
    }
    public int Size { get { return size + 1; } }
    public void Push(int id, float priority)
    {
        if (size < maxSize - 1)
        {
            size = size + 1;
            heap[size].id = id;
            heap[size].priority = priority;
            ShiftUp(size);

            hash[id] = true;
        }
        else
        {
            throw new InsufficientMemoryException();
        }
    }
    public int Pop()
    {
        PriorityQueueElement result = heap[0];

        heap[0] = heap[size];
        size = size - 1;
        ShiftDown(0);

        hash[result.id] = false;
        return result.id;
    }
    public int Peek()
    {
        return heap[0].id;
    }
    public bool Contains(int id)
    {
        return hash[id];
    }
    public void Clear()
    {
        size = -1;
        for (int i = 0; i < maxSize; i++) hash[i] = false;
    }
    public bool IsEmpty()
    {
        return size == -1;
    }
    int Parent(int i)
    {
        return (i - 1) / 2;
    }
    void ShiftUp(int i)
    {
        while (i > 0 && heap[Parent(i)].priority > heap[i].priority)
        {
            var tmp = heap[Parent(i)];
            heap[Parent(i)] = heap[i];
            heap[i] = tmp;

            i = Parent(i);
        }
    }
    void ShiftDown(int i)
    {
        int l = (2 * i) + 1;
        int r = l + 1;
        PriorityQueueElement tmp;
        while (l <= size && heap[l].priority < heap[i].priority ||
            r <= size && heap[r].priority < heap[i].priority)
        {
            if (r > size || heap[l].priority < heap[r].priority)
            {
                tmp = heap[l];
                heap[l] = heap[i];
                heap[i] = tmp;
                i = l;
            }
            else
            {
                tmp = heap[r];
                heap[r] = heap[i];
                heap[i] = tmp;
                i = r;
            }
            l = (2 * i) + 1;
            r = l + 1;
        }
    }
}
public interface IPathFinder
{
    public Stack<Vector2Int> FindPath(Vector2Int start, Vector2Int end);
}
public class AAStar: IPathFinder
{
    struct Node
    {
        public int previousID;
        public float distanceFromStart;
        public float distanceToEnd;
    }
    PriorityQueue1 openset;
    RectMapTopology map;
    Node[] costMap;
    int shapeX, mapLength;
    /// <param name="map">0-255 0=obstacle 1-255=the higher the slower movement on the block</param>
    public AAStar(RectMapTopology map)
    {
        this.map = map;
        shapeX = map.ShapeX;
        mapLength = map.Length;
        openset = new PriorityQueue1(mapLength);
        costMap = new Node[mapLength];
    }
    public int ToID(Vector2Int vec)
    {
        return vec.x + vec.y * shapeX;
    }
    public Vector2Int FromID(int id)
    {
        Vector2Int tmp = new Vector2Int();
        tmp.x = id % shapeX;
        tmp.y = id / shapeX;
        return tmp;
    } 
    /// <summary>
    /// Find a path from start block to end block using A*
    /// </summary>
    public Stack<Vector2Int> FindPath(Vector2Int start, Vector2Int end)
    {
        int startID = ToID(start), endID = ToID(end);
        int startX = start.x, startY = start.y, endX = end.x, endY = end.y;
        try
        {
            if (map[startID].Length == 0)
                return null; //if start point is not reachable, return
        }
        catch (ArgumentOutOfRangeException)
        {
            return null;
        }
        openset.Clear();
        for (int i = 0; i < mapLength; i++)
        {     
            costMap[i].previousID = -1;
            costMap[i].distanceFromStart = int.MaxValue;
            costMap[i].distanceToEnd = map.GetDistance(startX,startY,endX,endY);
        }
        costMap[startID].distanceFromStart = 0;
        openset.Push(startID, 0);
        float g, h;
        while (!openset.IsEmpty())
        {
            var current = openset.Pop();
            if (current == endID)
            {
                var pathStack = new Stack<Vector2Int>(64);
                pathStack.Push(FromID(current));
                while (costMap[current].previousID != -1)
                {
                    current = costMap[current].previousID;
                    pathStack.Push(FromID(current));
                }
                return pathStack; //success
            }

            foreach(MapTopologyConnection c in map[current])
            {
                g = costMap[current].distanceFromStart + c.distance;
                if(g<costMap[c.id].distanceFromStart)
                {
                    h = costMap[c.id].distanceToEnd;
                    costMap[c.id].previousID = current;
                    costMap[c.id].distanceFromStart = g;
                    if(!openset.Contains(c.id))
                    {
                        openset.Push(c.id, g + h);
                    }
                }
            }
        }
        
        return null;
    }
}


public struct MapTopologyConnection
{
    public int id;
    public float distance;
}
public abstract class RectMapTopology
{
    public abstract int ShapeX { get; }
    public abstract int Length { get; }
    public abstract float GetDistance(int id1, int id2);
    public abstract float GetDistance(Vector2Int start, Vector2Int end);
    public abstract float GetDistance(int startX, int startY, int endX, int endY);
    public abstract MapTopologyConnection[] this[int id] { get; }
}
public class RectMapTopology8: RectMapTopology
{
    //0 1 2
    //7   3
    //6 5 4
    private int shapeX,shapeY;
    private double D = 0, D2 = 0; // D2 is the mean distance of diagonal move, D is normal move
    private MapTopologyConnection[][] map;
    public RectMapTopology8(byte[,] map)
    {
        this.map = new MapTopologyConnection[map.Length][];
        shapeX = map.GetLength(0);
        shapeY = map.GetLength(1);
        byte[] tmp = new byte[8];
        int counter;
        long Dcounter = 0, D2counter = 0; //use the caculate D and D2
        for (int i = 0; i < shapeX; i++)
        {
            for (int j = 0; j < shapeY; j++)
            {
                if (map[i, j] == 0)
                {
                    this.map[i + j * shapeX] = new MapTopologyConnection[0];
                }
                else
                {
                    for (int k = 0; k < 8; k++) tmp[k] = 0; // Init tmp
                    //find valid connection
                    if (i > 0)
                    {
                        tmp[7] = map[i - 1, j];
                        if (j > 0) tmp[0] = map[i - 1, j - 1];
                        if (j < shapeX - 1) tmp[6] = map[i - 1, j + 1];
                    }
                    if (i < shapeX - 1)
                    {
                        tmp[3] = map[i + 1, j];
                        if (j > 0) tmp[2] = map[i + 1, j - 1];
                        if (j < shapeX - 1) tmp[4] = map[i + 1, j + 1];
                    }
                    if (j > 0) tmp[1] = map[i, j - 1];
                    if (j < shapeY - 1) tmp[5] = map[i, j + 1];
                    //caculate if diagonal valid due to the block on other directions
                    for (int k = 1; k < 8; k+=2)
                    {
                        if (tmp[k] == 0)
                        {
                            if (k == 7) tmp[0] = tmp[6] = 0;
                            else tmp[k + 1] = tmp[k - 1] = 0;
                        }
                    }
                    //caculate the required space to minimize the ram usage
                    counter = 0;
                    for (int k = 0; k < 8; k++)
                    {
                        if (tmp[k] != 0) counter++;
                    }
                    this.map[i + j * shapeX] = new MapTopologyConnection[counter];
                    //fill in the array
                    for (int k = 0, l = 0; k < 8; k++)
                    {
                        if (tmp[k] != 0)
                        {
                            if (k % 2 == 0)
                            {
                                this.map[i + j * shapeX][l].distance = (map[i, j] + tmp[k]) / 2f * 1.414213562373f;
                                D2 += this.map[i + j * shapeX][l].distance;
                                D2counter++;
                            }
                            else
                            {
                                this.map[i + j * shapeX][l].distance = (map[i, j] + tmp[k]) / 2f;
                                D += this.map[i + j * shapeX][l].distance;
                                Dcounter++;
                            }
                            switch (k)
                            {
                                case 0:
                                    this.map[i + j * shapeX][l].id = ((i - 1) + (j - 1) * shapeX);
                                    break;
                                case 1:
                                    this.map[i + j * shapeX][l].id = (i + (j - 1) * shapeX);
                                    break;
                                case 2:
                                    this.map[i + j * shapeX][l].id = ((i + 1) + (j - 1) * shapeX);
                                    break;
                                case 3:
                                    this.map[i + j * shapeX][l].id = ((i + 1) + j * shapeX);
                                    break;
                                case 4:
                                    this.map[i + j * shapeX][l].id = ((i + 1) + (j + 1) * shapeX);
                                    break;
                                case 5:
                                    this.map[i + j * shapeX][l].id = (i + (j + 1) * shapeX);
                                    break;
                                case 6:
                                    this.map[i + j * shapeX][l].id = ((i - 1) + (j + 1) * shapeX);
                                    break;
                                case 7:
                                    this.map[i + j * shapeX][l].id = ((i - 1) + j * shapeX);
                                    break;
                                default:
                                    break;
                            }
                            l++;
                        }
                    }

                }
            }
        }
        D /= Dcounter;
        D2 /= D2counter;
    }
    public override MapTopologyConnection[] this[int id]   {   get => map[id];   }
    public override int ShapeX { get => shapeX; }
    public override int Length { get => map.Length; }
    public override float GetDistance(int startID, int endID)
    {
        int dx = Math.Abs(startID % shapeX - endID % shapeX);
        int dy = Math.Abs(startID / shapeX - endID / shapeX);
        return (float)((dx + dy) + (D2 - 2 * D) * Math.Min(dx, dy));
    }
    public override float GetDistance(Vector2Int start, Vector2Int end)
    {
        int dx = Math.Abs(start.x - end.x);
        int dy = Math.Abs(start.y - end.y);
        return (float)((dx + dy) + (D2 - 2 * D) * Math.Min(dx, dy));
    }
    public override float GetDistance(int startX, int startY, int endX, int endY)
    {
        int dx = startX > endX ? startX - endX : endX - startX;
        int dy = startY > endY ? startY - endY : endY - startY;
        return (float)((dx + dy) + (D2 - 2 * D) * Math.Min(dx, dy));
    }
}
public class RectMapTopology4 : RectMapTopology
{
    private int shapeX, shapeY;
    private double D = 0; // D is mean distance normal move
    private MapTopologyConnection[][] map;
    public RectMapTopology4(byte[,] map)
    {
        this.map = new MapTopologyConnection[map.Length][];
        shapeX = map.GetLength(0);
        shapeY = map.GetLength(1);
        byte[] tmp = new byte[4];
        int counter;
        long Dcounter = 0; //use the caculate D and D2
        for (int i = 0; i < shapeX; i++)
        {
            for (int j = 0; j < shapeY; j++)
            {
                if (map[i, j] == 0)
                {
                    this.map[i + j * shapeX] = new MapTopologyConnection[0];
                }
                else
                {
                    for (int k = 0; k < 4; k++) tmp[k] = 0; // Init tmp
                    //find valid connection
                    if (i > 0)  tmp[0] = map[i - 1, j];
                    if (i < shapeX - 1) tmp[1] = map[i + 1, j];
                    if (j > 0) tmp[2] = map[i, j - 1];
                    if (j < shapeY - 1) tmp[3] = map[i, j + 1];
                    //caculate the required space to minimize the ram usage
                    counter = 0;
                    for (int k = 0; k < 4; k++)
                    {
                        if (tmp[k] != 0) counter++;
                    }
                    this.map[i + j * shapeX] = new MapTopologyConnection[counter];
                    //fill in the array
                    for (int k = 0, l = 0; k < 4; k++)
                    {
                        if (tmp[k] != 0)
                        {
                            this.map[i + j * shapeX][l].distance = (map[i, j] + tmp[k]) / 2f;
                            D += this.map[i + j * shapeX][l].distance;
                            Dcounter++;
                            switch (k)
                            {
                                case 0:
                                    this.map[i + j * shapeX][l].id = ((i - 1) + j * shapeX);
                                    break;
                                case 1:
                                    this.map[i + j * shapeX][l].id = ((i + 1) + j * shapeX);
                                    break;
                                case 2:
                                    this.map[i + j * shapeX][l].id = (i + (j - 1) * shapeX);
                                    break;
                                case 3:
                                    this.map[i + j * shapeX][l].id = (i + (j+1) * shapeX);
                                    break;
                                default:
                                    break;
                            }
                            l++;
                        }
                    }

                }
            }
        }
        D /= Dcounter;
    }
    public override MapTopologyConnection[] this[int id] { get => map[id]; }
    public override int ShapeX { get => shapeX; }
    public override int Length { get => map.Length; }
    public override float GetDistance(int startID, int endID)
    {
        return 0;
    }
    public override float GetDistance(Vector2Int start, Vector2Int end)
    {
        int dx = Math.Abs(start.x - end.x);
        int dy = Math.Abs(start.y - end.y);
        return dx+dy;
    }
    public override float GetDistance(int startX, int startY, int endX, int endY)
    {
        return (startX > endX ? startX - endX : endX - startX) + (startY > endY ? startY - endY : endY - startY);
    }
}