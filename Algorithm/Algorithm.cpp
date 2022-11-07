#include <bits/stdc++.h>
#define _GLIBCXX_DEBUG
using namespace std;

//bit全列挙
//使用する際はbitsetの<>内をNに変更すること(bit長は明示的に指定しないとエラーになるため)
//tmpは10進数で表現されているが、bitsetの初期化の際にそれを渡すことで、自動的にbit列に扱いを変えてくれる
//O(2^N)
//そのまま使用不可
void bit_enumerate(int N){
    for(int tmp = 0; tmp < (1 << N); tmp++){
        bitset<3> s(tmp);   //<>内を整数に変更して使用
        cout << s << endl;
    }
}

//部分和問題に対するbit全探索
//使用する際はbitsetの<>内をNに変更すること(bit長は明示的に指定しないとエラーになるため)
//tmpは10進数で表現されているが、bitsetの初期化の際にそれを渡すことで、自動的にbit列に扱いを変えてくれる
//Aは与えられる配列、NはAの要素数、Wは目的とする和の値
//O(2^N)
//そのまま使用不可
bool bit_all_search_for_partial_sum_problem(int N, vector<int> A, int W){
    bool ans = false;
    for(int tmp = 0; tmp < (1 << N); tmp++){
        bitset<3> s(tmp);   //<>内を整数に変更して使用
        int sum = 0;
        for(int i = 0; i < N; i++){   //各bit位置が1か0か判定する
            if(s.test(i)){
                sum += A[i];
            }
        }
        if(sum == W){
            ans = true;
            break;
        }
    }
    return ans;
}

//GCD(Greatest common divisor:最大公約数)に対する再帰関数
//nの方が大きかったとしても、次の再帰で呼ばれるのはGCD(n, m)となるので問題ない
//m ÷ n = shou ... r
//という上記の関係がある場合、mとnの最大公約数は、nとrの最大公約数と等しい、という性質を利用
//O(log2 N) (m >= n > 0の場合)
//そのまま使用可能
int GCD(int m, int n){
    if(n == 0) return 0;
    return GCD(n, m%n);
}

//フィボナッチ数列を求める動的計画法(メモ化再帰)
//memoはメモ化用配列でここではグローバル変数として定義しているが、main関数内で定義しても良い
//main関数内で求めたい項の値で読んだ後、memoの中身をfor等で取り出す
//O(N) (Nは求めたい項の値)
//そのまま使用不可
vector<long long> memo(50, -1); //例として50としているが、求めたい項の値に変更
long long fibo(int N){
    if(N == 0) return 0;
    else if(N == 1) return 1;
    else if(memo[N] != -1) return memo[N];
    else return memo[N] = fibo(N - 1) + fibo(N - 2);   //メモ化しながらreturn
}

//昇順にソート済み配列の中から値がkeyとなる要素の添字を求める二分探索法
//発見できればindexを、見つからなければ-1を返却
//そのまま使用可能(vector Aの宣言文だけ削除)
//O(log2 N)
vector<int> A(10); //例として置いてあるだけで、mainとかで定義してソート済みにしておきこの宣言は不要
int binary_search_for_vector_index(int key){
    int left = 0, right = (int)A.size() - 1;
    while(right >= left){
        int mid = left + (right - left) / 2;
        if(A[mid] == key) return mid;   //A[mid]がkeyならば終了
        else if(A[mid] > key) right = mid - 1;   //探索範囲をmidの左側として再探索
        else if(A[mid] < key) left = mid + 1;   //探索範囲をmidの右側として再探索
    }
    return -1; //見つからなかったら-1を返却
}

//左からある条件に対してfalse領域とtrue領域を持つ連続した整数列に対する一般化した二分探索法
//true領域の一番左端の添字(right)を返却
//つまりここでは、Judge(x) = trueとなる最小のxを返却する
//そのまま使用不可(Judge関数の内部を記述する)
//O(log2 N)
bool Judge(int x){
    //xが条件を満たすかどうか
}

int general_binary_search_for_calculate_first_index_of_true_region(){
    int left, right;
    while(right - left > 1){   
        int mid = left + (right - left) / 2;
        if(Judge(mid)) right = mid;
        else left = mid;
    }
    return right;
}

//Frog問題に対する動的計画法(配る遷移方式, 貰う遷移方式)
//Frog問題:N個の足場があってその高さはh[i]で与えられており、現在カエルは0番目の足場にいて、N-1番目の足場を目指す
// 遷移はiからi+1にコスト|h[i]-h[i-1]|を払うか、iからi+2にコスト|h[i]-h[i-2]|を払うことで可能
//chmin関数は、緩和処理用
//そのまま使用不可(main1,2関数のうち好きな遷移方式を使用)
//O(N)
template<class T> void chmin(T&a, T b){
    if(a > b) a = b;
}
const long long INF = (1 << 60); //(=2^60)
//1.配る遷移方式
int main1(){
    int N;
    cin >> N;
    vector<long long> h(N);   //コスト
    for(int i = 0; i < N; i++) cin >> h[i];
    vector<long long> dp(N, INF);
    dp[0] = 0;   //初期条件
    for(int i = 0; i < N; i++){
        if(i + 1 < N) chmin(dp[i+1], dp[i] + abs(h[i] - h[i+1]));
        if(i + 2 < N) chmin(dp[i+2], dp[i] + abs(h[i] - h[i+2]));
    }
    cout << dp[N-1] << endl;
}
//2.貰う遷移方式
int main2(){
    int N;
    cin >> N;
    vector<long long> h(N);
    for(int i = 0; i < N; i++) cin >> h[i];
    vector<long long> dp(N, INF);
    dp[0] = 0;
    for(int i = 1; i < N; i++){
        chmin(dp[i], dp[i-1] + abs(h[i] - h[i-1]));
        if(i > 1) chmin(dp[i], dp[i-2] + abs(h[i] - h[i-2]));   //i=1の時だけi-2がないため
    }
    cout << dp[N-1] << endl;
}

//ナップサック問題に対する動的計画法
//ナップサック問題:N個の品物があり、i(=0, 1, ..., N-1)番目の品物の重さはweight[i]、価値はvalue[i]で与えられる
// このN個の品物の中から、重さの総和がWを超えないように、品物を選んでいき、選んだ品物の価値の総和の最大値を求める
// dp[i][w]を、最初のi個の品物{0, 1, ..., i-1}までの中から重さがwを超えないように選んだ時の価値の最大値としてテーブルを定義
//そのまま使用可能
//O(NW)
template<class T> void chmax(T& a, T b){
    if(a < b) a = b;
}
int main3(){
    int N;
    long long W;
    cin >> N >> W;
    vector<long long> weight(N), value(N);
    for(int i = 0; i < N; i++) cin >> weight[i] >> value[i];
    vector<vector<long long>> dp(N+1, vector<long long>(W+1, 0));   //N+1なのは何も選ばないN=0の初期状態を含めるため、W+1なのはw=0の初期状態含めるため
    for(int i = 0; i < N; i++){
        for(int w = 0; w < W; w++){
            if(w - weight[i] >= 0) chmax(dp[i+1][w], dp[i][w-weight[i]] + value[i]);   //i番目の品物を選ぶのは、wにまだ余裕がある時だけ 
            chmax(dp[i+1][w], dp[i][w]);   //選ばない場合には、その前の最大価値を流用できる
        }
    }
    cout << dp[N][W] << endl;   //求めたいのはこれだけ
}

//区間分割の仕方を最適化する問題に対する動的計画法
//区間分割の仕方を最適化する問題:
// N個の要素が一列に並んでいて、これを幾つかの区間に分割する
// 各区間[l, r)にはスコアc(l, r)が付いている
// KをN以下の正の整数として、K+1個の整数t0, t1, ..., tKを
// 0 = t0 < t1 < t2 < ... < tK = N
// を満たすようにとったとき、区間分割[t0, t1), [t1, t2), ..., [tK-1, tK)のスコアを
// c(t0, t1) + c(t1, t2) + ... + c(tK-1, tK)
// と定義したとき、N要素の区間分割の仕方を全て考慮したとき最小となるスコアを求める
// |0 1 2| 3 4| 5| 6 7 8 9| 10| みたいな感じにtはそれぞれの仕切りに当たる 
//dp[i]は、区間[0, i)について、いくつかの区間に分割する最小コストとして定義
//そのまま使用可能
//O(N^2)
template<class T> void chmin(T& a, T b){
    if(a > b) a=b;
}
const long long INF = 1LL << 60;
int main4(){
    int N;
    cin >> N;
    vector<vector<long long>> c(N+1, vector<long long>(N+1));   //N+1なのは、[0, 0)という0個の要素を幾つかの区間に分割するという初期状態を含める為
    for(int i = 0; i < N + 1; i++){
        for(int j = 0; j < N + 1; j++) cin >> c[i][j];
    }
    vector<long long> dp(N+1, INF);
    dp[0] = 0;
    for(int i = 0; i < N + 1; i++){
        for(int j = 0; j < i; j++) chmin(dp[i], dp[j] + c[j][i]);
    }
    cout << dp[N] << endl;
}

//小数点第N位を四捨五入する(0 < N)
//考え方:四捨五入をする桁を小数点第一位となるように10の累乗を掛ける
// その値に対して0.5を足した後、int型にキャストして小数点以下を切り捨てる
// 最後に掛けた10の累乗で割れば終了
//(decimal:少数の, decimal place:少数位)
//そのまま使用可能
//O(1)
double round_off_the_number_to_N_1th_decimal_places(double x, int N){
    double ans = int(x * pow(10, N-1) + 0.5) / pow(10, N-1);
    return ans;
}

//10^Nの位以下を四捨五入する(0 <= N)
//考え方:上記の小数点第N位を四捨五入する方法の逆をすればOK
// まず先に5*10^Nを加えてから四捨五入する位に5を加える
// そして、四捨五入する位が小数点第一位に来るように10の累乗で割る
// それをint型にキャストして小数点第一位を切り捨ててから、先ほど使用した10の累乗を掛けて位の位置を戻す
//そのまま使用可能
//O(1)
int round_off_the_number_to_the_nearest_pow_10_N1(int x, int N){
    int ans = int((x + 5 * pow(10, N)) / pow(10, N+1)) * pow(10, N+1);
    return ans;
}