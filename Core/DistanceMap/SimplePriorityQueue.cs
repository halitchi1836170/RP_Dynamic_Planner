using System;
using System.Collections.Generic;

public class SimplePriorityQueue<TElement, TPriority> where TPriority:IComparable<TPriority>
{
    private readonly List<(TElement Element, TPriority Priority)> _elements = new List<(TElement, TPriority)>();

    public int Count => _elements.Count;

    public void Enqueue(TElement element, TPriority priority)
    {
        _elements.Add((element, priority));
        HeapifyUp(_elements.Count - 1);
    }

    public bool TryDequeue(out TElement element, out TPriority priority)
    {
        if (_elements.Count == 0)
        {
            element = default;
            priority = default;
            return false;
        }

        element = _elements[0].Element;
        priority = _elements[0].Priority;

        int lastIndex = _elements.Count - 1;
        _elements[0] = _elements[lastIndex];
        _elements.RemoveAt(lastIndex);

        if (_elements.Count > 0)
        {
            HeapifyDown(0);
        }

        return true;
    }

    private void HeapifyUp(int index)
    {
        var item = _elements[index];
        while (index > 0)
        {
            int parentIndex = (index - 1) / 2;
            var parent = _elements[parentIndex];

            // Se la priorità del nodo corrente è maggiore o uguale a quella del padre, fermati
            if (item.Priority.CompareTo(parent.Priority) >= 0)
                break;

            _elements[index] = parent;
            index = parentIndex;
        }
        _elements[index] = item;
    }

    private void HeapifyDown(int index)
    {
        int lastIndex = _elements.Count - 1;
        var item = _elements[index];

        while (true)
        {
            int leftChildIndex = index * 2 + 1;
            if (leftChildIndex > lastIndex)
                break;

            int rightChildIndex = leftChildIndex + 1;
            int minChildIndex = leftChildIndex;

            // Trova il figlio minore
            if (rightChildIndex <= lastIndex &&
                _elements[rightChildIndex].Priority.CompareTo(_elements[leftChildIndex].Priority) < 0)
            {
                minChildIndex = rightChildIndex;
            }

            // Se l'elemento corrente è minore o uguale al figlio minore, il min-heap è valido
            if (item.Priority.CompareTo(_elements[minChildIndex].Priority) <= 0)
                break;

            _elements[index] = _elements[minChildIndex];
            index = minChildIndex;
        }
        _elements[index] = item;
    }
}
