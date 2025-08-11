<<<<<<< HEAD
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

//編集距離に対する動的計画法
//編集距離:
//  二つの文字列S,Tが与えられる
//  Sに以下の三通りの操作を繰り返すことでTに変換したい
//  そのような一連の操作のうち、操作回数の最小値を求める
//  この最小値をSとTの編集距離と呼ぶ
//  ・変更:S中の文字を一つ選んで任意の文字に変更する
//  ・削除:S中の文字を一つ選んで削除する
//  ・挿入:Sの好きな箇所に好きな文字を一文字挿入する
//そのまま使用可能
//O(|S| * |T|)
template<class T> coid chmin(T& a, T b){
    if(a > b) a = b;
}
const int INF = 1 << 29;
int main10(){
    string S, T;
    cin >> S >> T;
    vector<vector<int>> dp(S.size()+1, vector<int>(T.size()+1, INF));
    dp[0][0] = 0;
    for(int i = 0; i <= S.size(); i++){
        for(int j = 0; j <= T.size(); j++){
            if(i > 0 && j > 0){
                if(S[i-1] == T[j-1]) chmin(dp[i][j], dp[i-1][j-1]);
                else chmin(dp[i][j], dp[i-1][j-1] + 1);
            }
            if(i > 0) chmin(dp[i][j], dp[i-1][j] + 1);
            if(j > 0) chmin(dp[i][j], dp[i][j-1] + 1);
        }
    }
    cout << dp[S.size()][T.size()] << endl;
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

//ペア和の最適化問題に対する二分探索法
//ペア和の最適化問題:
//  N個の整数a0, a1, ..., aNと、N個の整数b0, b1, ..., bNが与えられたとき、二組の整数列からそれぞれ一つずつ選んで和をとる
//  この時その和として考えられる値のうち、整数K以上の範囲内での最小値を求める問題
//  ただし、a[i]+b[j] >= Kを満たすような整数i, jの組が少なくとも一つは存在するとする
//そのまま使用可能
//O(Nlog2N)
const int INF = 20000000;
int main5(){
    int N, K;
    cin >> N >> K;
    vector<int> a(N), b(N);
    for(int i = 0; i < N; i++) cin >> a[i];
    for(int i = 0; i < N; i++) cin >> b[i];
    int min_value = INF;
    sort(b.begin(), b.end());
    b.push_back(INF);   //b.end()はbの要素の次のアドレスを指すイテレータであり、これにアクセスすると未定義の動作を引き起こすため追加する
    for(int i = 0; i < N; i++){   //a[i]を選ぶ
        auto iter = lower_bound(b.begin(), b.end(), K - a[i]);   //a[i]を選んだ時にK-a[i]以上の最小のbのイテレータを求める
        int value = *iter;   //イテレータの指す値へのアクセス
        if(a[i] + value < min_value) min_value = a[i] + value;   //最小でなかった場合だけでなく、そもそもK-a[i]以上の値が存在しないときもここで破棄される
    }
    cout << min_value << endl;
}

//N個の値の最大値を最小にするという最適化問題に対する二分探索法
//N個の値の最大値を最小にするという最適化問題:
//  N個の風船があり、それぞれ初期状態ではh[i]の高さにあり、一秒ごとにs[i]だけ上昇する。
//  このような風船を射撃によって割る
//  0秒後に1個風船を割ることができ、そこから一秒ごとに一個の風船を割ることができ、最終的に全ての風船を割る
//  順番は自由に選ぶことが可能
//  風船を割るのに必要なペナルティ(値)はその時の風船の高度として、N個の風船を割り終わった時のコストとして考えられる最小値を求める問題
//そのまま使用可能
//O(N*log2N*log2N) (ただし、M = max(h[0]+N*s[0], h[1]+N*s[i], ..., h[N-1]+N*s[N-1]))
const long long INF = 1LL << 60;
int main6(){
    int N;
    cin >> N;
    vector<long long> h(N), s(N);
    for(int i = 0; i < N; i++) cin >> h[i] >> s[i];
    long long left = 0, right = INF;   //探索範囲はペナルティ直線上で、全ての風船を割ることが可能な最小のペナルティを求める
    while(right - left > 1){
        long long mid = (left + right) / 2;
        bool frug = true;
        vector<long long> t(N, 0);   //あと何秒以内に風船を割らなければならないかを保持する配列
        for(int i = 0; i < N; i++){
            if(mid < h[i]) frug = false;   //そもそもmidが初期高度より低かったらfalse
            else t[i] = (mid - h[i]) / s[i];   //これは切り捨てで問題ない。例えば残り時間が2秒の時に、midに到達するのが1.5秒後だろうと1秒後でもt[i](=1, 1.5) < 2が成立するため
        }
        sort(t.begin(), t.end());   //時間制限が短い順にソート
        for(int i = 0; i < N; i++){
            if(t[i] < i) frug = false;   //制限時間(t[i])が、現在時刻から考えた実現可能な時刻(i)よりも小さければ割れない
        }
        if(frug) right = mid;   //ペナルティ直線上ではmidはtrue領域にあるので、midの左側でfalse領域とtrue領域の境界を探す
        else left = mid;   //ペナルティ直線上ではmidはfalse領域にあるので、midの右側でfalse領域とtrue領域の境界を探す
    }
    cout << right << endl;
}

//コイン問題に対する貪欲法
//コイン問題:
//  500,100,50,10,5,1円玉がa[0], a[1], ..., a[5]枚ある
//  これらを用いてX円を支払いたい
//  この時、支払いに用いるコインの枚数を最小にする問題
//そのまま使用可能
//O(1)
const vector<int> coins = {500, 100, 50, 10, 5, 1};
int main7(){
    int X;
    cin >> X;
    vector<int> a(6);
    for(int i = 0; i < 6; i++) cin >> a[i];

    int ans = 0;
    for(int i = 0; i < 6; i++){
        int num = X / coins[i];
        if(num > a[i]) num = a[i];
        X -= coins[i] * num;
        ans += num;
    }
    cout << ans << endl;
}

//区間スケジューリング問題に対する貪欲法
//区間スケジューリング問題:
//  N個の仕事があり、i番目の仕事は時刻interval[i].firstに開始され、interval[i].secondに終了する
//  この時、時刻が重ならないように仕事を選ぶとき、選ぶ仕事数の最大値を求める問題
//そのまま使用可能
//O(N*log2N)
using Interval = pair<int, int>;   //エイリアス宣言
bool compare(const Interval &a, const Interval &b){   //比較関数
    return a.second < b.second;
}
int main8(){
    int N;
    cin >> N;
    vector<Interval> interval(N);
    for(int i = 0; i < N; i++) cin >> interval[i].first >> interval[i].second;   //firstが区間の開始時間, secondが区間の終了時間
    sort(interval.begin(), interval.end(), compare);   //区間の終了時刻でソート
    int count = 0;   //ans
    int current_end_time = 0;   //現在選択した最新の区間の終了時間
    for(int i = 0; i < N; i++){
        if(interval[i].first < current_end_time) continue;   //これまでに選択した区間の終了時間よりも開始時間の早い全ての区間は無視し、終了時間よりも遅くて開始時間が一番早いものを選択している
        count++;
        current_end_time = interval[i].second;
    }
    cout << count << endl;
}

//ボタンを押してA[i]をB[i]の倍数にする問題に対する貪欲法
//ボタンを押してA[i]をB[i]の倍数にする問題:
//  0以上の整数からなるN項の数列A[0], A[1], ..., A[N-1]とN個のボタンiが与えられる
//  i番目のボタンを押すと、A[0], A[1], ..., A[i]の値がそれぞれ1ずつ増加する
//  一方、1以上の整数からなるN項の数列B[0], B[1], ..., B[N-1]が与えられる
//  ボタンを何回か押して、全てのiに対してA[i]がB[i]の倍数になるようにしたい
//  この時、ボタンを押す回数の最小値を求める問題
//  そのまま使用可能
//O(N)
int main9(){
    int N;
    cin >> N;
    vector<long long> A(N), B(N);
    for(int i = 0; i < N; i++) cin >> A[i] >> B[i];
    long long count = 0;   //ans
    for(int i = N - 1; i >= 0; i--){   //i=N-1から始める
        A[i] += count;   //それまでにボタンを押された回数分だけ加算
        long long remainder = A[i] % B[i];
        long long D = 0;
        if(remainder != 0) D = B[i] - remainder;   //最小の回数でA[i]をB[i]の倍数にする
        count++;
    } 
    cout << count << endl; 
=======
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

//編集距離に対する動的計画法
//編集距離:
//  二つの文字列S,Tが与えられる
//  Sに以下の三通りの操作を繰り返すことでTに変換したい
//  そのような一連の操作のうち、操作回数の最小値を求める
//  この最小値をSとTの編集距離と呼ぶ
//  ・変更:S中の文字を一つ選んで任意の文字に変更する
//  ・削除:S中の文字を一つ選んで削除する
//  ・挿入:Sの好きな箇所に好きな文字を一文字挿入する
//そのまま使用可能
//O(|S| * |T|)
template<class T> coid chmin(T& a, T b){
    if(a > b) a = b;
}
const int INF = 1 << 29;
int main10(){
    string S, T;
    cin >> S >> T;
    vector<vector<int>> dp(S.size()+1, vector<int>(T.size()+1, INF));
    dp[0][0] = 0;
    for(int i = 0; i <= S.size(); i++){
        for(int j = 0; j <= T.size(); j++){
            if(i > 0 && j > 0){
                if(S[i-1] == T[j-1]) chmin(dp[i][j], dp[i-1][j-1]);
                else chmin(dp[i][j], dp[i-1][j-1] + 1);
            }
            if(i > 0) chmin(dp[i][j], dp[i-1][j] + 1);
            if(j > 0) chmin(dp[i][j], dp[i][j-1] + 1);
        }
    }
    cout << dp[S.size()][T.size()] << endl;
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

//ペア和の最適化問題に対する二分探索法
//ペア和の最適化問題:
//  N個の整数a0, a1, ..., aNと、N個の整数b0, b1, ..., bNが与えられたとき、二組の整数列からそれぞれ一つずつ選んで和をとる
//  この時その和として考えられる値のうち、整数K以上の範囲内での最小値を求める問題
//  ただし、a[i]+b[j] >= Kを満たすような整数i, jの組が少なくとも一つは存在するとする
//そのまま使用可能
//O(Nlog2N)
const int INF = 20000000;
int main5(){
    int N, K;
    cin >> N >> K;
    vector<int> a(N), b(N);
    for(int i = 0; i < N; i++) cin >> a[i];
    for(int i = 0; i < N; i++) cin >> b[i];
    int min_value = INF;
    sort(b.begin(), b.end());
    b.push_back(INF);   //b.end()はbの要素の次のアドレスを指すイテレータであり、これにアクセスすると未定義の動作を引き起こすため追加する
    for(int i = 0; i < N; i++){   //a[i]を選ぶ
        auto iter = lower_bound(b.begin(), b.end(), K - a[i]);   //a[i]を選んだ時にK-a[i]以上の最小のbのイテレータを求める
        int value = *iter;   //イテレータの指す値へのアクセス
        if(a[i] + value < min_value) min_value = a[i] + value;   //最小でなかった場合だけでなく、そもそもK-a[i]以上の値が存在しないときもここで破棄される
    }
    cout << min_value << endl;
}

//N個の値の最大値を最小にするという最適化問題に対する二分探索法
//N個の値の最大値を最小にするという最適化問題:
//  N個の風船があり、それぞれ初期状態ではh[i]の高さにあり、一秒ごとにs[i]だけ上昇する。
//  このような風船を射撃によって割る
//  0秒後に1個風船を割ることができ、そこから一秒ごとに一個の風船を割ることができ、最終的に全ての風船を割る
//  順番は自由に選ぶことが可能
//  風船を割るのに必要なペナルティ(値)はその時の風船の高度として、N個の風船を割り終わった時のコストとして考えられる最小値を求める問題
//そのまま使用可能
//O(N*log2N*log2N) (ただし、M = max(h[0]+N*s[0], h[1]+N*s[i], ..., h[N-1]+N*s[N-1]))
const long long INF = 1LL << 60;
int main6(){
    int N;
    cin >> N;
    vector<long long> h(N), s(N);
    for(int i = 0; i < N; i++) cin >> h[i] >> s[i];
    long long left = 0, right = INF;   //探索範囲はペナルティ直線上で、全ての風船を割ることが可能な最小のペナルティを求める
    while(right - left > 1){
        long long mid = (left + right) / 2;
        bool frug = true;
        vector<long long> t(N, 0);   //あと何秒以内に風船を割らなければならないかを保持する配列
        for(int i = 0; i < N; i++){
            if(mid < h[i]) frug = false;   //そもそもmidが初期高度より低かったらfalse
            else t[i] = (mid - h[i]) / s[i];   //これは切り捨てで問題ない。例えば残り時間が2秒の時に、midに到達するのが1.5秒後だろうと1秒後でもt[i](=1, 1.5) < 2が成立するため
        }
        sort(t.begin(), t.end());   //時間制限が短い順にソート
        for(int i = 0; i < N; i++){
            if(t[i] < i) frug = false;   //制限時間(t[i])が、現在時刻から考えた実現可能な時刻(i)よりも小さければ割れない
        }
        if(frug) right = mid;   //ペナルティ直線上ではmidはtrue領域にあるので、midの左側でfalse領域とtrue領域の境界を探す
        else left = mid;   //ペナルティ直線上ではmidはfalse領域にあるので、midの右側でfalse領域とtrue領域の境界を探す
    }
    cout << right << endl;
}

//コイン問題に対する貪欲法
//コイン問題:
//  500,100,50,10,5,1円玉がa[0], a[1], ..., a[5]枚ある
//  これらを用いてX円を支払いたい
//  この時、支払いに用いるコインの枚数を最小にする問題
//そのまま使用可能
//O(1)
const vector<int> coins = {500, 100, 50, 10, 5, 1};
int main7(){
    int X;
    cin >> X;
    vector<int> a(6);
    for(int i = 0; i < 6; i++) cin >> a[i];

    int ans = 0;
    for(int i = 0; i < 6; i++){
        int num = X / coins[i];
        if(num > a[i]) num = a[i];
        X -= coins[i] * num;
        ans += num;
    }
    cout << ans << endl;
}

//区間スケジューリング問題に対する貪欲法
//区間スケジューリング問題:
//  N個の仕事があり、i番目の仕事は時刻interval[i].firstに開始され、interval[i].secondに終了する
//  この時、時刻が重ならないように仕事を選ぶとき、選ぶ仕事数の最大値を求める問題
//そのまま使用可能
//O(N*log2N)
using Interval = pair<int, int>;   //エイリアス宣言
bool compare(const Interval &a, const Interval &b){   //比較関数
    return a.second < b.second;
}
int main8(){
    int N;
    cin >> N;
    vector<Interval> interval(N);
    for(int i = 0; i < N; i++) cin >> interval[i].first >> interval[i].second;   //firstが区間の開始時間, secondが区間の終了時間
    sort(interval.begin(), interval.end(), compare);   //区間の終了時刻でソート
    int count = 0;   //ans
    int current_end_time = 0;   //現在選択した最新の区間の終了時間
    for(int i = 0; i < N; i++){
        if(interval[i].first < current_end_time) continue;   //これまでに選択した区間の終了時間よりも開始時間の早い全ての区間は無視し、終了時間よりも遅くて開始時間が一番早いものを選択している
        count++;
        current_end_time = interval[i].second;
    }
    cout << count << endl;
}

//ボタンを押してA[i]をB[i]の倍数にする問題に対する貪欲法
//ボタンを押してA[i]をB[i]の倍数にする問題:
//  0以上の整数からなるN項の数列A[0], A[1], ..., A[N-1]とN個のボタンiが与えられる
//  i番目のボタンを押すと、A[0], A[1], ..., A[i]の値がそれぞれ1ずつ増加する
//  一方、1以上の整数からなるN項の数列B[0], B[1], ..., B[N-1]が与えられる
//  ボタンを何回か押して、全てのiに対してA[i]がB[i]の倍数になるようにしたい
//  この時、ボタンを押す回数の最小値を求める問題
//  そのまま使用可能
//O(N)
int main9(){
    int N;
    cin >> N;
    vector<long long> A(N), B(N);
    for(int i = 0; i < N; i++) cin >> A[i] >> B[i];
    long long count = 0;   //ans
    for(int i = N - 1; i >= 0; i--){   //i=N-1から始める
        A[i] += count;   //それまでにボタンを押された回数分だけ加算
        long long remainder = A[i] % B[i];
        long long D = 0;
        if(remainder != 0) D = B[i] - remainder;   //最小の回数でA[i]をB[i]の倍数にする
        count++;
    } 
    cout << count << endl; 
>>>>>>> 1663d07cc1a7def70a0d9547740e84a4a1979add
}