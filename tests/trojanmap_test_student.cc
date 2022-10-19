#include <vector>
#include <string>
#include <regex>
#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

//Phase 1
TEST(TrojanMapTest, Autocomplete1) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("Good");
  std::unordered_set<std::string> gt = {"Good News Missionary Baptist Church", "Good News Missionary Baptist Church 1"};
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("good");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case 
  names = m.Autocomplete("GOoD"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case 
  names = m.Autocomplete("GOOD"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}
TEST(TrojanMapTest, Autocomplete2) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("Che");
  std::unordered_set<std::string> gt = {"Chevron", "Chevron 1","Cheebos Burger","Chevron 2"};
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("che");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case 
  names = m.Autocomplete("cHe"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case 
  names = m.Autocomplete("CHE"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}
TEST(TrojanMapTest, Autocomplete3) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("New");
  std::unordered_set<std::string> gt = {"New Light Baptist Church", "New Zion Baptist Church","Newton Community Police Station","Newman Recital Hall in Hancock Foundation","New Orleans Fish Market","New Residential College"};
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("new");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case 
  names = m.Autocomplete("NeW"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case 
  names = m.Autocomplete("NEW"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}
TEST(TrojanMapTest, FindPosition123) {
  TrojanMap a;
  
  // Test Chick-fil-A
  auto position = a.GetPosition("Chevron");
  std::pair<double, double> gt1(34.0106834, -118.2822420); // groundtruth for "Chick-fil-A"
  EXPECT_EQ(position, gt1);
  // Test Ralphs
  position = a.GetPosition("Chevron 1");
  std::pair<double, double> gt2(34.0336409, -118.2620798); // groundtruth for "Ralphs"
  EXPECT_EQ(position, gt2);
  // Test Target
  position = a.GetPosition("Chevron 2");
  std::pair<double, double> gt3(34.0357551, -118.2913828); // groundtruth for "Target"
  EXPECT_EQ(position, gt3);
  // Test Unknown
  position = a.GetPosition("XXX");
  std::pair<double, double> gt4(-1, -1);
  EXPECT_EQ(position, gt4);
}

TEST(TrojanMapTest, CalculateEditDistance123) {
  TrojanMap c;
  EXPECT_EQ(c.CalculateEditDistance("apple", "amole"), 2);
  EXPECT_EQ(c.CalculateEditDistance("banana", "banbnb"), 2);
  EXPECT_EQ(c.CalculateEditDistance("peach", "peh"), 2);
}

TEST(TrojanMapTest, FindClosestName123) {
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName("Shelll"), "Shell");
  EXPECT_EQ(m.FindClosestName("Chvron"), "Chevron");
  EXPECT_EQ(m.FindClosestName("766"), "66");
}

TEST(TrojanMapTest, Regex1){
  TrojanMap f;
  std::vector<std::pair<double, double>> baba={{34.0357551, -118.2913828},{34.0336409, -118.2620798}};
  std::regex r("Chevron [0-9]*");
  std::vector<std::pair<double, double>> caca;
  caca = f.GetLocationRegex(r);
  EXPECT_EQ(caca, baba);
}
TEST(TrojanMapTest, Regex2){
  TrojanMap f;
  std::vector<std::pair<double, double>> baba={};
  std::regex r("Shellfgdgd");
  std::vector<std::pair<double, double>> caca;
  caca = f.GetLocationRegex(r);
  EXPECT_EQ(caca, baba);
}
TEST(TrojanMapTest, Regex3){
  TrojanMap k;
  std::vector<std::pair<double, double>> baba={{34.0169985,-118.2822768}};
  std::regex r("Chi[a-z]*");
  std::vector<std::pair<double, double>> caca;
  caca = k.GetLocationRegex(r);
  EXPECT_EQ(caca, baba);
}

TEST(TrojanMapTest, CalculateShortestPath_Dijkstra1) {
  TrojanMap m;

  auto path = m.CalculateShortestPath_Dijkstra("Bank of America", "Chase");
  std::vector<std::string> gt{ "5237417651", "6813360960", "6814620882", "6813360954", "6813360952", "6813379420", "6813360951", "6813360936", "6813379467", "6813379439", "544671962", "2783295153", "6813379408", "9591449452", "6807200381", "6813379456", "6818390146", "3872400990", "3402917921", "3402917919", "9559739236", "6045067409", "3398574883", "9559739232", "9591449441" }; // Expected path

  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  

  path = m.CalculateShortestPath_Dijkstra("Chase","Bank of America");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra2) {
  TrojanMap m;

  auto path = m.CalculateShortestPath_Dijkstra("USC Village Gym", "Ramen KenJo");
  std::vector<std::string> gt{ "6045040487", "5567729100", "6047234443", "6813379419", "9591449515", "6814820015", "6047198312", "6814820008", "5237417652", "6813379473" }; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  path = m.CalculateShortestPath_Dijkstra("Ramen KenJo","USC Village Gym");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra3) {
  TrojanMap m;

  auto path = m.CalculateShortestPath_Dijkstra("Lyons Center","USC Village Gym");
  std::vector<std::string> gt{ "6042952605", "6814770362", "123346391", "6808200835", "6808200834", "6808200837", "6808200833", "4091160832", "4091160831", "4091160830", "4091160823", "3402887079", "6818390141", "6818390142", "6818390139", "6818390143", "6045054380", "6813379418", "6047234443", "5567729100", "6045040487" };// Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  path = m.CalculateShortestPath_Dijkstra("USC Village Gym","Lyons Center");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford1) {
  TrojanMap m;

  auto path = m.CalculateShortestPath_Bellman_Ford("Lyons Center","USC Village Gym");
  std::vector<std::string> gt{ "6042952605", "6814770362", "123346391", "6808200835", "6808200834", "6808200837", "6808200833", "4091160832", "4091160831", "4091160830", "4091160823", "3402887079", "6818390141", "6818390142", "6818390139", "6818390143", "6045054380", "6813379418", "6047234443", "5567729100", "6045040487" }; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  

  path = m.CalculateShortestPath_Bellman_Ford("USC Village Gym","Lyons Center");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford2) {
  TrojanMap m;

  auto path = m.CalculateShortestPath_Bellman_Ford("Leavey Library","Trojan Grounds (Starbucks)");
  std::vector<std::string> gt{ "6042978413", "1673645147", "6814770350", "1673644992", "614990288" }; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  

  path = m.CalculateShortestPath_Bellman_Ford("Trojan Grounds (Starbucks)","Leavey Library");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
TEST(TrojanMapTest, CycleDetection) {
  TrojanMap m;
  
  // Test case 1
  std::vector<double> square1 = {-118.299, -118.264, 34.032, 34.011};
  auto sub1 = m.GetSubgraph(square1);
  bool result1 = m.CycleDetection(sub1, square1);
  EXPECT_EQ(result1, true);

  // Test case 2
  std::vector<double> square2 = {-118.290, -118.289, 34.030, 34.020};
  auto sub2 = m.GetSubgraph(square2);
  bool result2 = m.CycleDetection(sub2, square2);
  EXPECT_EQ(result2, false);

  // Test case 3
  std::vector<double> square3 = {-119, -118, 34, 33};
  auto sub3 = m.GetSubgraph(square3);
  bool result3 = m.CycleDetection(sub3, square3);
  EXPECT_EQ(result3, true);

}

TEST(TrojanMapTest, TopologicalSort1) {
  TrojanMap m;
  
  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","KFC"}, {"Chick-fil-A","Ralphs"}, {"KFC","Chick-fil-A"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> bala={};
  EXPECT_EQ(result,bala);
}
TEST(TrojanMapTest, TopologicalSort2) {
  TrojanMap m;
  
  std::vector<std::string> location_names = {"USC Village Gym", "USC Parking", "Shell"};
  std::vector<std::vector<std::string>> dependencies = {{"Shell","USC Parking"}, {"USC Parking","USC Village Gym"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Shell", "USC Parking","USC Village Gym"};
  EXPECT_EQ(result, gt);
}
TEST(TrojanMapTest, TopologicalSort3) {
  TrojanMap n;
  
  std::vector<std::string> location_names = {"The Caribbean Apartments", "USC Village Gym","USC Parking", "Shell","Ralphs", "Chick-fil-A", "KFC","Chase","Bank of America","Statbucks"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","KFC"}, {"Ralphs","Shell"}, {"KFC","Chick-fil-A"},{"KFC","Chase"},{"Chase","USC Parking"},{"The Caribbean Apartments","Shell"},
  {"The Caribbean Apartments","USC Village Gym"},{"Shell","USC Parking"},{"USC Parking","USC Village Gym"},{"Bank of America","Chick-fil-A"},{"USC Parking","Statbucks"}};
  auto result = n.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={ "Ralphs", "The Caribbean Apartments", "KFC", "Shell", "Chase", "USC Parking", "Bank of America", "USC Village Gym", "Statbucks", "Chick-fil-A" };
  EXPECT_EQ(result, gt);
}


TEST(TrojanMapTest, TSP1) {
  TrojanMap m;
  
  std::vector<std::string> input{"732641023","6045040487","9591449441","5237417650","3810143804","6042952605","6813379473","4399693647"}; // Input location ids 
  auto result = m.TravelingTrojan_Brute_force(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"732641023","9591449441","3810143804","5237417650","6813379473","6045040487","6042952605","4399693647","732641023"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest, TSP2) {
  TrojanMap m;
  
  std::vector<std::string> input{"3088548446","3577173167","5695183056","7159563415"}; // Input location ids 
  auto result = m.TravelingTrojan_Backtracking(input);
  // for(auto &val : result.second){
  //   for(auto &i : val) printf("%s\n",i.c_str());
  // } 
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"3088548446","7159563415","3577173167","5695183056","3088548446"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest, TSP3) {
  TrojanMap m;
  
  std::vector<std::string> input{"5237417651","9591449441","2134538676","2556689683","358828099","358850013"}; // Input location ids 
  auto result = m.TravelingTrojan_2opt(input);
  // for(auto &val : result.second){
  //   for(auto &i : val) printf("%s\n",i.c_str());
  // } 
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"5237417651","9591449441","2134538676","2556689683","358850013","358828099","5237417651"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, FindNearby1) {
  TrojanMap m;
  
  auto result = m.FindNearby("bank", "Chase", 10, 0);
  std::vector<std::string> ans{};
  EXPECT_EQ(result, ans);
}
TEST(TrojanMapTest, FindNearby2) {
  TrojanMap l;
  
  auto result = l.FindNearby("convenience", "Lees Market", 10, 5);
  std::vector<std::string> ans{ "7158037351", "5695236165", "3577173162", "3800035750", "4630579202" };
  EXPECT_EQ(result, ans);
}
TEST(TrojanMapTest, FindNearby3) {
  TrojanMap o;
  
  auto result = o.FindNearby("pharmacy", "CVS Pharmacy", 10, 5);
  std::vector<std::string> ans{ "3577173167", "5695183056", "7159563415" };
  EXPECT_EQ(result, ans);
}