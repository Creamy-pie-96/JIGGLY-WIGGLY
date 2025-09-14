#include <iostream>
#include <vector>
#include <algorithm> // Include algorithm for sort
#include <climits>
using namespace std;

int main()
{
    ios::sync_with_stdio(false);
    cin.tie(0);

    int t;
    cin >> t;
    while (t--)
    {
        long long n;
        cin >> n;
        vector<long long> solutions;

        long long pow10 = 10;
        for (int k = 1;; ++k)
        {
            long long d = pow10 + 1;
            if (d > n)
                break; // Stop if 10^k + 1 > n
            if (n % d == 0)
                solutions.push_back(n / d);
            if (pow10 > LLONG_MAX / 10)
                break; // safety
            pow10 *= 10;
        }

        if (solutions.empty())
        {
            cout << 0 << '\n';
        }
        else
        {
            sort(solutions.begin(), solutions.end());
            cout << solutions.size() << '\n';
            for (size_t i = 0; i < solutions.size(); ++i)
            {
                if (i)
                    cout << ' ';
                cout << solutions[i];
            }
            cout << '\n';
        }
    }

    return 0;
}
