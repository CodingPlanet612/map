#include "trojanmap.h"

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return
 * -1.
 *
 * @param  {std::string} id : location? id
 * @return {double}         : latitude
 * WM
 */
double TrojanMap::GetLat(const std::string &id) { 
    if (data.find(id) != data.end()) return data[id].lat;
    return -1;
  }


/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist,
 * return -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 * WM
 */
double TrojanMap::GetLon(const std::string &id) { 
  if (data.find(id) != data.end()) return data[id].lon;
  return -1;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return
 * "NULL".
 *
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 * WM
 */
std::string TrojanMap::GetName(const std::string &id) { 
  if (data.find(id) != data.end()) return data[id].name;
  return "NULL";
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return
 * an empty vector.
 *
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 * WM
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string &id) {
  if (data.find(id) != data.end()) return data[id].neighbors;
  return {};
}

/**
 * GetID: Given a location name, return the id.
 * If the node does not exist, return an empty string.
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 * WM
 **/
std::string TrojanMap::GetID(const std::string &name) {
  std::string res = "";
  for (auto &it : data) {
    if (GetName(it.first) == name) {
      res = it.second.id;
      return res;
    }
  }
  return res;
}

/**
 * GetPosition: Given a location name, return the position. If id does not
 * exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 * WM
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {

  std::pair<double, double> results(-1, -1);
  std::string tempforid;
  tempforid = GetID(name);
  if (tempforid.size()==0) return results;
  results.first = data[tempforid].lat;
  results.second = data[tempforid].lon;

  return results;
}

/**
 * CalculateEditDistance: Calculate edit distance between two location names
 *WM
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b) { 
  int m=a.size(),n=b.size();
  int dp[m + 1][n + 1];

  for (int i = 0; i <= m; i++ ){
    for (int j = 0; j <= n; j++ ){
      if (i == 0) 
        dp[i][j] = j; 
      else if (j == 0) 
        dp[i][j] = i;
      else{ 
        if (tolower(a[i - 1]) == tolower(b[j - 1]))
            dp[i][j] = dp[i - 1][j - 1];
        else
            dp[i][j] = 1 + std::min({dp[i][j - 1],      // Insert
                                     dp[i - 1][j],      // Remove
                                     dp[i - 1][j - 1]}); // Replace
      }
    }
  }
  return dp[m][n];
}


/**
 * FindClosestName: Given a location name, return the name with smallest edit
 * distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : similar name
 * WM
 */
std::string TrojanMap::FindClosestName(std::string name) {
  std::string ans = "";
  int min = INT_MAX;
  int tmp_distance;
  for (auto &it: data) {
    tmp_distance = CalculateEditDistance(name, it.second.name);
    if (tmp_distance < min) {
      ans = it.second.name;;
      min = tmp_distance;
    }
  }
  return ans;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 * WM
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name) {
  std::vector<std::string> results;
  for (auto &it : data)
  {
    int cnt = 0;
    std::string &s = it.second.name;
    if (s.size() < name.size()) 
      continue;
    for (int j = 0; j < name.size(); j++)
    {
      if (tolower(s[j]) == tolower(name[j]))
        cnt++;
      else
        break;
    }
    if (cnt == name.size())
      results.push_back(s);
  }

  return results;
}

/**
 * GetAllCategories: Return all the possible unique location categories, i.e.
 * there should be no duplicates in the output.
 *
 * @return {std::vector<std::string>}  : all unique location categories
 * WM
 */
std::vector<std::string> TrojanMap::GetAllCategories() {
  std::vector<std::string> ans;
  std::unordered_set<std::string> check;
  for (auto &it: data) {
    auto tmp = it.second.attributes;
    
    if (tmp.size() != 0) 
      for (auto iter = tmp.begin(); iter != tmp.end(); iter++) {
        if (check.find(*iter) == check.end()) {
          check.insert(*iter);
          ans.push_back(*iter);
        }
      }
  }
  return ans;
}

/**
 * GetAllLocationsFromCategory: Return all the locations of the input category (i.e.
 * 'attributes' in data.csv). If there is no location of that category, return
 * (-1, -1). The function should be case-insensitive.
 *
 * @param  {std::string} category          : category name (attribute)
 * @return {std::pair<double, double>}     : (lat, lon)
 * WM
 */
std::vector<std::pair<double, double>> TrojanMap::GetAllLocationsFromCategory(
    std::string category) {
          std::vector<std::pair<double, double>> a;
    int cnt=0;
    int CateSize = category.size();

    for (auto &it : data){ // std::unordered_map<std::string, Node> data;
      for (auto &p : it.second.attributes){ // std::unordered_set<std::string>attributes;
        if (p.size() != CateSize) continue;
        for (int j = 0; j < CateSize; j++){
          if (tolower(p[j]) == tolower(category[j]))
            cnt++;
        }
        if (cnt == CateSize){
          a.emplace_back(it.second.lat, it.second.lon);
          cnt = 0;
          break;
        }
      }
    }

    return a;
    
    }

/**
 * GetLocationRegex: Given the regular expression of a location's name, your
 * program should first check whether the regular expression is valid, and if so
 * it returns all locations that match that regular expression.
 *
 * @param  {std::regex} location name      : the regular expression of location
 * names
 * @return {std::pair<double, double>}     : (lat, lon)
 * WM
 */
std::vector<std::pair<double, double>> TrojanMap::GetLocationRegex(std::regex location) {
  std::vector<std::pair<double, double>> MatchRegex;
  for (auto it : data){ 
    if (std::regex_match(GetName(it.first),location)) 
        MatchRegex.push_back(std::make_pair(it.second.lat, it.second.lon));
  }
  //for (auto it : MatchRegex) printf("%f,%f",it.first,it.second);
  return MatchRegex;
}

/**
 * CalculateDistance: Get the distance between 2 nodes.
 *
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 * Tea
 */
double TrojanMap::CalculateDistance(const std::string &a_id,
                                    const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2), 2.0) + cos(a.lat * M_PI / 180.0) *
                                           cos(b.lat * M_PI / 180.0) *
                                           pow(sin(dlon / 2), 2.0);
  double c = 2 * asin(std::min(1.0, sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations
 * inside the vector.
 *
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 * Tea
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0; i < int(path.size()) - 1; i++) {
    sum += CalculateDistance(path[i], path[i + 1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path
 * which is a list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 * WM
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
  typedef std::pair<double, std::string> wmPair;
  std::vector<std::string> path; 
  std::priority_queue <wmPair, std::vector<wmPair>, std::greater<wmPair>> pq;
  std::unordered_map<std::string, double> d;
  std::unordered_set<std::string> visited;
  std::unordered_map<std::string, std::string> prev;
  
  std::string Start = GetID(location1_name); 
  std::string Goal = GetID(location2_name);

  pq.push(make_pair(0, Start));
  d[Start] = 0;
  visited.insert(Start);

  while (visited.find(Goal) == visited.end()) {
    std::string visiting = pq.top().second;
    pq.pop();
    visited.insert(visiting);

    for (auto &n: GetNeighborIDs(visiting)) {
      if (visited.find(n) == visited.end()) {
        double weight = CalculateDistance(visiting, n);

        if (d.find(n) == d.end()) {  //node n has not been updated before
          d.insert(make_pair(n, d[visiting] + weight));
          pq.push(make_pair(d[n], n));
          prev[n] = visiting;
        } else {  //update node n
          if (d[n] > (d[visiting] + weight)) {
            d[n] = d[visiting] + weight;
            pq.push(make_pair(d[n], n));
            prev[n] = visiting;
            }
          }
      }
    }
  }

  std::string u = Goal;

  while (u != Start) {
    path.push_back(u);
    u = prev[u];
  }
  path.push_back(Start);
  std::reverse(path.begin(), path.end());
  return path;
}

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest
 * path which is a list of id. Hint: Do the early termination when there is no
 * change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;
  std::unordered_map<std::string, double> d;
  std::unordered_map<std::string, std::string> prev;

  std::string Start = GetID(location1_name); 
  std::string Goal = GetID(location2_name);

  for (auto &n: data)  // data: A map of ids to Nodes.
    d[n.first] = (n.first == Start) ? 0 : DBL_MAX;

  // bool stop = false;
  // while (!stop) {
  // //for (int i = 0; i < data.size()-1; i++) {
  //   stop = true;
  //   for (auto &n : data) {
  //     if (n.first != Start) {
  //       for (auto &m : n.second.neighbors) {
  //         if (d[m] != DBL_MAX) 
  //           if (d[n.first] > d[m] + CalculateDistance(n.first, m)) {
  //             d[n.first] = d[m] + CalculateDistance(n.first, m);
  //             prev[n.first] = m;
  //             stop = false;
  //           }
  //       }
  //     }
  //   }
  // }

  // faster version
  std::queue<std::string> q;
  q.push(Start);
  while(!q.empty()){
    auto &now=data[q.front()];q.pop();
    for(auto &to:now.neighbors){
      double dis=CalculateDistance(now.id,to);
      if(d[to]>d[now.id]+dis){
        d[to]=d[now.id]+dis;
        prev[to]=now.id;
        q.push(to);
      }
    }
  }

  if(d[Goal]==DBL_MAX) return path;
  std::string u = Goal;
  while (u != Start) {
    path.push_back(u);
    u = prev[u];
  }
  path.push_back(Start);
  std::reverse(path.begin(), path.end());

  return path;
}


/**
 * Traveling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of
 * total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>>
TrojanMap::TravelingTrojan_Brute_force(std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;

  std::vector<std::vector<std::string>> temp; //  all
  std::vector<std::string> order; //  single compare
  std::vector<int> p; //  for permutation

  const int n=location_ids.size();
  double dis[n][n]; // save sll the distances
  records.first = DBL_MAX;

  for(int i = 0; i < n; i++) {//save distance
    for(int j = i; j < n; j++){
      double distance = CalculateDistance(location_ids[i],location_ids[j]);
      dis[i][j] = dis[j][i] = distance;
    }
  }

  for(int i = 0; i < n; i++) p.push_back(i);//{0,1,2,3,4,5...}

  do {
    double Dis = 0;

    for(int i=0; i < n - 1; i++)
      Dis += dis[p[i]][p[i+1]];
    Dis += dis[p[n-1]][p[0]];

    if (records.first > Dis){
      records.first = Dis;
      order.clear();
      for(int i = 0; i < n; i++)
        order.push_back(location_ids[p[i]]);
      order.push_back(order[0]);
      records.second.emplace_back(order);
    }
    
  }while(std::next_permutation(p.begin()+1,p.end()));

  return records;
}

void TrojanMap::dfs_helper(int now,int visited[],int a[],std::vector<std::string> &location_ids,
    std::pair<double, std::vector<std::vector<std::string>>> &records,double cur){

  double Dis;
  int n = location_ids.size();
        
  if(now == n){//reach the bottom
    cur += CalculateDistance(location_ids[a[n-1]],location_ids[0]);
    records.first = std::min(records.first,cur);
    //printf("%f",records.first);
    if(records.first == cur){
      std::vector<std::string> tmp;
      for(int i=0;i<n;i++)  tmp.push_back(location_ids[a[i]]);
      tmp.push_back(location_ids[0]);
      records.second.push_back(tmp);
      //for(auto bal : tmp) printf("%s\n",bal.c_str());
      //printf("\n");
    } 
    return;
}

  for(int i = 0;i < n; i++)if(!visited[i]){
    Dis = CalculateDistance(location_ids[a[now-1]],location_ids[i]);
    if(Dis + cur >= records.first) continue;

    visited[i]=1;
    a[now]=i;//

    dfs_helper(now+1,visited,a,location_ids,records,cur+Dis);
    visited[i]=0;
    //break;
    }
  }
std::pair<double, std::vector<std::vector<std::string>>>
TrojanMap::TravelingTrojan_Backtracking(std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records; 
  int visited[location_ids.size()+1];
  int a[location_ids.size()+1];
  records.first = DBL_MAX;
  visited[0]=1;
  for(int i =1;i<location_ids.size();i++) visited[i]=0;
  a[0]=0;
  dfs_helper(1,visited,a,location_ids,records,0);
  // for(auto val : a){
  //   printf("%d\n",val);
  // } 
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>>
TrojanMap::TravelingTrojan_2opt(std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;

  const int size = location_ids.size();
  std::vector<std::string> order;
  double dis[size][size]; // save sll the distances
  records.first = DBL_MAX;
  double new_distance = 0;
  std::vector<int> p;
  std::vector<int> tmp;
  std::vector<int> tmp2;

  for(int i = 0; i < size; i++) { // save distance
    for(int j = i; j < size; j++) {
      double distance = CalculateDistance(location_ids[i],location_ids[j]);
      dis[i][j] = dis[j][i] = distance;
    }
  }
 
  double tmp_dis = DBL_MAX;
  int next_node;
  int cur_node= 0;
  int count = 0;
  std::unordered_set<int> visited;
  while (count < size) {
    p.push_back(cur_node);
    visited.insert(cur_node);
    for (int j = 0; j < size; j++) {
      if ((dis[cur_node][j] < tmp_dis) && (visited.find(j) == visited.end())) 
        next_node = j;
    }
    cur_node = next_node;
    count++;
  }

  // for (int j = 0; j<size; j++) { std::cout<<p[j];}
  // std::cout<<std::endl;

  for(int j = 0; j < size - 1; j++) new_distance += dis[p[j]][p[j + 1]]; 
  new_distance += dis[p[0]][p[size - 1]];
  records.first = new_distance;
  order.clear();
  for(int j = 0; j < size; j++) order.push_back(location_ids[p[j]]);
  order.push_back(order[0]);
  records.second.emplace_back(order);
  
  // for (int j = 0; j<order.size(); j++) { std::cout<<order[j]<<std::endl;}
  // std::cout<<std::endl;

  bool change = true;
  tmp = p;

  while (change) {
    change = false;
    for (int i = 1; i < size - 1; i++) {
      for (int k = i + 1; k < size; k++) {
        tmp2 = tmp;
        reverse(tmp2.begin()+i, tmp2.begin() + k + 1);

        // for (int j = 0; j<size; j++) std::cout<<tmp[j];
        // std::cout<<std::endl;

        new_distance = 0;
        for (int j = 0; j < size - 1; j++) {
          new_distance += dis[tmp2[j]][tmp2[j + 1]]; 
        }
        new_distance += dis[tmp2[0]][tmp2[size - 1]];
        if (new_distance < records.first) { // Improvement found so reset
          records.first = new_distance;
          order.clear();
          tmp = tmp2;
          change = true;
          for(int j = 0; j < size; j++)
            order.push_back(location_ids[tmp[j]]);
          order.push_back(order[0]);
          records.second.emplace_back(order);
          // for (int j = 0; j<size; j++) std::cout<<tmp[j];
          // std::cout<<std::endl;
          // for (int j = 0; j<order.size(); j++) { std::cout<<order[j]<<std::endl;}
          // std::cout<<std::endl;
        }
      }
      p.clear();
    }
  }

  return records;
}

std::pair<double, std::vector<std::vector<std::string>>>
TrojanMap::TravelingTrojan_3opt(std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;

  const int size = location_ids.size();
  std::vector<std::string> order;
  double dis[size][size]; // save sll the distances
  records.first = DBL_MAX;
  std::vector<int> p;
  std::vector<int> tmp;
  std::vector<int> tmp2;
  double new_distance = 0;

  for(int i = 0; i < size; i++) { // save distance
    for(int j = i; j < size; j++) {
      double distance = CalculateDistance(location_ids[i],location_ids[j]);
      dis[i][j] = dis[j][i] = distance;
    }
  }

  //for(int i = 0; i < size; i++) p.push_back(i); //{0,1,2,3,4,5...}

  double tmp_dis = DBL_MAX;
  int next_node;
  int cur_node= 0;
  int count = 0;
  std::unordered_set<int> visited;
  while (count < size) {
    p.push_back(cur_node);
    visited.insert(cur_node);
    for (int j = 0; j < size; j++) {
      if ((dis[cur_node][j] < tmp_dis) && (visited.find(j) == visited.end())) 
        next_node = j;
    }
    cur_node = next_node;
    count++;
  }

  // for (int j = 0; j<size; j++) { std::cout<<p[j];}
  // std::cout<<std::endl;

  for(int j = 0; j < size - 1; j++) new_distance += dis[p[j]][p[j + 1]]; 
  new_distance += dis[p[0]][p[size - 1]];
  records.first = new_distance;
  order.clear();
  for(int j = 0; j < size; j++) order.push_back(location_ids[p[j]]);
  order.push_back(order[0]);
  records.second.emplace_back(order);

  // std::cout<< "original ";
  // for (int j = 0; j<size; j++) std::cout << p[j];
  //   std::cout<<std::endl;

  bool change = true;
  tmp = p;

  while (change) {
    change = false;

    for (int x1 = 0; x1 < size - 2; x1++)
      for (int x2 = x1 + 1; x2 < size - 1; x2++)
        for (int x3 = x2 + 1; x3 < size; x3++) {
          int y1 = x1 + 1; 
          int y2 = x2 + 1;
          int y3 = x3 + 1;
          for (int k = 0; k < 7; k++) {
            tmp2 = tmp; 
            switch(k) {
              case 0:
              reverse(tmp2.begin() + y1, tmp2.begin() + y3);
              // std::cout<< "case0 ";
              //   for (int j = 0; j<size; j++) std::cout <<tmp[j];
              // std::cout<<std::endl;
              break;
              case 1:
              reverse(tmp2.begin() + y1, tmp2.begin() + y2);
              // std::cout<< "case1 ";
              //   for (int j = 0; j<size; j++) std::cout <<tmp[j];
              // std::cout<<std::endl;
              break;
              case 2:
              reverse(tmp2.begin() + y2, tmp2.begin() + y3);
              // std::cout<< "case2 ";
              //   for (int j = 0; j<size; j++) std::cout <<tmp[j];
              // std::cout<<std::endl;
              break;
              case 3:
              reverse(tmp2.begin() + y1, tmp2.begin() + y2);
              // reverse(tmp.begin() + y2, tmp.begin() + y3);
              // std::cout<< "case3 ";
              //   for (int j = 0; j<size; j++) std::cout <<tmp[j];
              // std::cout<<std::endl;
              break;
              case 4:
              reverse(tmp2.begin() + y1, tmp2.begin() + y3);
              reverse(tmp2.begin() + y3 - y2 + y1, tmp2.begin() + y3);
              // std::cout<< "case4 ";
              //   for (int j = 0; j<size; j++) std::cout <<tmp[j];
              // std::cout<<std::endl;
              break;
              case 5:
              reverse(tmp2.begin() + y1, tmp2.begin() + y3);
              reverse(tmp2.begin() + y1, tmp2.begin() + y1 + y3 - y2);
              // std::cout<< "case5 ";
              //   for (int j = 0; j<size; j++) std::cout <<tmp[j];
              // std::cout<<std::endl;
              break;
              case 6:
              reverse(tmp2.begin() + y1, tmp2.begin() + y3);
              reverse(tmp2.begin() + y1, tmp2.begin() + y1 + y3 - y2);
              reverse(tmp2.begin() + y3 - y2 + y1, tmp2.begin() + y3);
              // std::cout<< "case6 ";
              //   for (int j = 0; j<size; j++) std::cout <<tmp[j];
              // std::cout<<std::endl;
              // break;
            }

          new_distance = 0;
          for (int j = 0; j < size - 1; j++) 
            new_distance += dis[tmp2[j]][tmp2[j + 1]]; 
          new_distance += dis[tmp2[0]][tmp2[size - 1]];
          if (new_distance < records.first) { // Improvement found so reset
            records.first = new_distance;
            order.clear();
            change = true;
            tmp = tmp2;
            for(int j = 0; j < size; j++)
              order.push_back(location_ids[tmp[j]]);
            order.push_back(order[0]);
            records.second.emplace_back(order);

            // for (int j = 0; j<size; j++) std::cout<<tmp[j];
            // std::cout<<std::endl; }
          }
        }
    }
  }
  
  return records;
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(
  std::string locations_filename) {
  std::vector<std::string> location_names_from_csv;
  // freopen(locations_filename.c_str(), "r", stdin);
  //   const int BUFFER_SIZE = 256;
  //   char s[BUFFER_SIZE], r[BUFFER_SIZE];
  //   int id, amount;
  //   while (~scanf("%d%s%s%d", &id, s, r, &amount))
  //       trans.emplace_back(id, s, r, amount);
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(
    std::string dependencies_filename) {
  std::vector<std::vector<std::string>> dependencies_from_csv;
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a
 * sorting of nodes that satisfies the given dependencies. If there is no way to
 * do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     :
 * prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(
    std::vector<std::string> &locations,
    std::vector<std::vector<std::string>> &dependencies) {
  std::vector<std::string> result;
  std::map <std::string,int> table;
  std::queue<std::string> q;
  std::vector<std::vector<std::string>> de = dependencies;

  for (auto &name : locations){ //use table to store the in value
    table.insert(make_pair(name,0));
    for (auto &p : de){
      if(p[0] == name) table[name]++;
    } 
  }
  
  for(auto &val : table){
    if(val.second == 0) q.push(val.first);
  }
  while (!q.empty())
  {
    std::string cur = q.front();
    result.push_back(cur);
    q.pop();
  
    for(auto &p : de){
      if(p[1] == cur && (table[p[0]]!=0)){
        table[p[0]]--;
        if(table[p[0]]==0) q.push(p[0]);
      }  
    }
  }
  if(result.size() !=locations.size()) result={};
  reverse(result.begin(), result.end());
  return result;
}

/**
 * inSquare: Give a id return whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 * WM
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  if(GetLat(id)>square[0] && GetLat(id)<square[1] 
    &&GetLon(id)>square[2] && GetLon(id)<square[3])
      return true;
  return false;
}

/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location
 * ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square
 * area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the
 * square
 * WM
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;
  for (auto &it : data) {
    if (GetLon(it.first)>square[0] && 
        GetLon(it.first)<square[1] && 
        GetLat(it.first)>square[3] && 
        GetLat(it.first)<square[2]) {
      subgraph.push_back(it.first);
    }
  }
  return subgraph;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true
 * if there is a cycle path inside the square, false otherwise.
 *
 * @param {std::vector<std::string>} subgraph: list of location ids in the
 * square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 * WM
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph,
                               std::vector<double> &square) {
  std::unordered_map<std::string, bool> visited;
  std::unordered_set<std::string> idSet(subgraph.begin(),subgraph.end());
  std::map<std::string,std::string> parent;
  std::queue<std::string> q;

  for (auto &now : subgraph)
  {
    if (visited[now]) continue;
    q.push(now);
    visited[now] = true;
    while (!q.empty())
    {
      std::string cur = q.front();
      q.pop();
      for (auto &to : GetNeighborIDs(cur))
      {
        if (!idSet.count(to)) continue;
        if (visited[to])
        {
          if(parent[cur] != to) return true;
          continue;
        }
        visited[to] = true;
        q.push(to);
        parent[to]=cur;
      }
    }
  }

  return false;
}

/**
 * FindNearby: Given a class name C, a location name L and a number r,
 * find all locations in class C on the map near L with the range of r and
 * return a vector of string ids
 *
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {int} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName,
                                               std::string name, double r,
                                               int k) {
  std::vector<std::string> res;
  std::map< double,std::string> restemp;
  double dis;
  int cnt=0;

  for(auto &it : data){
    for(auto attnames : it.second.attributes){
        if (attnames == attributesName) {
          dis = CalculateDistance(GetID(name),it.first);
          if(r> dis && GetName(it.first)!=name){
              restemp.insert(make_pair(dis,it.first));
          }
       }
   }
  }
  for (auto &i :restemp){
    if(cnt<k){
      res.push_back(i.second);
      cnt++;
    } 
  }
  return res;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 *Tea
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0])) n.attributes.insert(word);
        if (isdigit(word[0])) n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}
