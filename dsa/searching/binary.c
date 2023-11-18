#include <stdio.h>

int main()
{
    int key;
    int arr[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    int low, mid, high, x;
    scanf("%d", &key);
    int found = 0;
    low = 0;
    high = 10;
    while (low <= high)
    {
        mid = (low + high) / 2;
        if (key < arr[mid])
        {
            high = mid - 1;
        }
        else if (key > arr[mid])
        {
            low = mid + 1;
        }
        else
        {
            printf("Found location %d\n", mid);
            found = 1;
            break;
        }
    }
    if (found == 0)
    {
        printf("element not found\n");
    }

    return 0;
}