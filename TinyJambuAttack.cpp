#include <iostream>
#include "gurobi_c++.h"
#include<bitset>
#include<map>
#include <fstream>
#include <string>

using namespace std;

const int MAX = 200000000; // the maximum value of PoolSearchMode, P625

//Regard two 256-bit vectors a and b as ordinary integers and compare them to see whether a<b
struct cmpBitset256 {
    bool operator()(const bitset<256>& a, const bitset<256>& b) const {
        for (int i = 0; i < 256; i++) {
            if (a[i] < b[i])
                return true;
            else if (a[i] > b[i])
                return false;
        }
        return false;
    }
};

GRBVar COPY(GRBModel& model, GRBVar& x) {

    GRBVar y = model.addVar(0, 1, 0, GRB_BINARY);
    GRBVar z = model.addVar(0, 1, 0, GRB_BINARY);
    GRBVar tmp[2] = { y,z };
    model.addGenConstrOr(x, tmp, 2);
    x = z; //x is replaced by z
    return y; //return y for other operations

}
void AND(GRBModel& model, vector<GRBVar>& lhs, GRBVar& rhs)
{
	//MILP model for AND
	//lhs->(AND)rhs
	for (int i = 0; i < lhs.size(); i++)
	{
		model.addConstr(rhs == lhs.at(i));
	}
}
void XOR(GRBModel& model, vector<GRBVar>& lhs, GRBVar& rhs)
{
	//MILP model for XOR
	//lhs->(XOR)rhs
	GRBLinExpr sum;

	for (int i = 0; i < lhs.size(); i++)
	{
		sum += lhs.at(i);
	}
	model.addConstr(rhs == sum);
}

void tinyjambuCore(GRBModel& model, vector<GRBVar>& x)
{
    //this function takes the model and a 256 bit vector where 0~127 is the state and 128~256 is the key
    GRBVar a = model.addVar(0, 1, 0, GRB_BINARY); //for a=s70*s85
    GRBVar y0 = model.addVar(0, 1, 0, GRB_BINARY);
    GRBVar y47 = model.addVar(0, 1, 0, GRB_BINARY);
    GRBVar y70 = model.addVar(0, 1, 0, GRB_BINARY);
    GRBVar y85 = model.addVar(0, 1, 0, GRB_BINARY);
    GRBVar y91 = model.addVar(0, 1, 0, GRB_BINARY);
    GRBVar y128 = model.addVar(0, 1, 0, GRB_BINARY);

    //GRBVar z0 = model.addVar(0, 1, 0, GRB_BINARY); //no need z0, since need to assign back
    GRBVar z47 = model.addVar(0, 1, 0, GRB_BINARY);
    //GRBVar z70 = model.addVar(0, 1, 0, GRB_BINARY); //no need z70 and z85, since z70=z85=a
    //GRBVar z85 = model.addVar(0, 1, 0, GRB_BINARY); 
    GRBVar z91 = model.addVar(0, 1, 0, GRB_BINARY);
    GRBVar z128 = model.addVar(0, 1, 0, GRB_BINARY);

    //COPY y47
    model.addConstr(y47 <= x[47]);
    model.addConstr(z47 <= x[47]);
    model.addConstr(y47 + z47 >= x[47]);

    //COPY y70
    model.addConstr(y70 <= x[70]);
    model.addConstr(a <= x[70]);
    model.addConstr(y70 + a >= x[70]);

    //COPY y85
    model.addConstr(y85 <= x[85]);
    model.addConstr(a <= x[85]);
    model.addConstr(y85 + a >= x[85]);

    //COPY y91
    model.addConstr(y91 <= x[91]);
    model.addConstr(z91 <= x[91]);
    model.addConstr(y91 + z91 >= x[91]);

    //COPY y128
    model.addConstr(y128 <= x[128]);
    model.addConstr(z128 <= x[128]);
    model.addConstr(y128 + z128 >= x[128]);

    //XOR
    model.addConstr(y0 == x[0] + z47 + a + z91 + z128);

    x[0] = y0;
    x[47] = y47;
    x[70] = y70;
    x[85] = y85;
    x[91] = y91;
    x[128] = y128;
}

int tinyjambuThreeEnumeration(vector<int> cube, int rounds, map<bitset<256>, int, cmpBitset256>& countingBox, int threadnum)
{
    try {
        GRBEnv env = GRBEnv();
        env.set(GRB_IntParam_LogToConsole, 0);
        env.set(GRB_IntParam_PoolSearchMode, 2);//focus on finding best solutions 
        env.set(GRB_IntParam_PoolSolutions, MAX); // try to find 2000000
        env.set(GRB_IntParam_MIPFocus, GRB_MIPFOCUS_BESTBOUND);
        env.set(GRB_IntParam_Threads, threadnum);
        
        GRBModel model = GRBModel(env);

        vector<GRBVar> s(256); // the initial register
        for (int i = 0; i < 256; i++)
            s[i] = model.addVar(0, 1, 0, GRB_BINARY); // declare binary variables

        for (int i = 0; i < 96; i++)
            model.addConstr(s[i] == 0); // constant 0

        for (int i = 96; i < 128; i++) { // IVs are all public variables
            if (cube[i - 96] == 1)
            {
                //cube bits are set to 1, else set to 0
                model.addConstr(s[i] == 1);
                
            }
            else
            {
                model.addConstr(s[i] == 0);
                cout << "excluding Cube index is: " << i-96 << endl;
            }
                
        }

        vector<GRBVar> works = s;
        cout << "Permutation for " << rounds << " rounds" << endl;
        for (int r = 0; r < rounds; r++)
        {
            //cout << "in for loop" << endl;
            tinyjambuCore(model, works);

            vector<GRBVar> temp = works;
            for (int i = 0; i < 128; i++) //shifting the states
                works[i] = temp[(i + 1) % 128];
            for (int i = 128; i < 255; i++) //shifting the key
                works[i] = temp[i + 1];
            works[255] = temp[128];
        }

        // output function

        for (int i = 0; i < 127; i++)
            model.addConstr(works[i] == 0);

        GRBLinExpr nk = works[127];
        for (int i = 128; i < 256; i++)
            model.addConstr(works[i] == 0);

        
        model.addConstr(nk == 1);

        // the degree 
        GRBLinExpr nv = 0;
        for (int i = 128; i < 256; i++) //looking for the largest term consisting of key variables
            nv += s[i];

        // max the degree as the objective function
        model.setObjective(nv, GRB_MAXIMIZE);


        //following is solving MILP model

        model.update();
        model.optimize();
        cout << "After optimized" << endl;


        int solCount = model.get(GRB_IntAttr_SolCount);
        double dulation = model.get(GRB_DoubleAttr_Runtime);
        
        // check solution limit
        if (solCount >= 2000000000) {
            cerr << "Number of solutions is too large" << endl;
            exit(0);
        }

        // store the information about solutions
        for (int i = 0; i < solCount; i++) {
            model.set(GRB_IntParam_SolutionNumber, i);
            bitset<256> tmp;
            for (int j = 0; j < 256; j++) {
                if (round(s[j].get(GRB_DoubleAttr_Xn)) == 1) tmp[j] = 1;
                else tmp[j] = 0;
            }
            countingBox[tmp]++;
        }

        // display result
        auto it = countingBox.begin();
        while (it != countingBox.end()) {

            cout << ((*it).second % 2) << " | " << (*it).second << "\t";
            bitset<256> tmp = (*it).first;
            for (int i = 128; i < 256; i++) {
                if ((tmp[i] == 1)) {
                    cout << "k" << (i - 128) << " ";
                }
            }
            for (int i = 96; i < 128; i++) {
                if ((tmp[i] == 1) && (cube[i - 96] == 0)) {
                    cout << "v" << (i - 96) << " ";
                }
            }
            cout << endl;

            it++;
        }

        cout << dulation << "sec" << endl;

        //result
        if (model.get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
            return -1;
        }
        else if ((model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)) {
            int upperBound = round(model.get(GRB_DoubleAttr_ObjVal));
            return upperBound;
        }
        else {
            cout << model.get(GRB_IntAttr_Status) << endl;
            return -2;
        }
    }
    catch (GRBException e) {
        cerr << "Error code = " << e.getErrorCode() << endl;
        cerr << e.getMessage() << endl;
    }
    catch (...) {
        cerr << "Exception during optimization" << endl;
    }
}

void attack(int excludeCubeIndex, int evalRounds=400)
{
    vector<int> cube(32, 1); //32 bit nonce set to 1 first
    if (excludeCubeIndex>=0&&excludeCubeIndex<=31)
    {
        cube[excludeCubeIndex] = 0; //exclude 1 nonce bit from the cube
    }
    int start=0, end=0;
    if (evalRounds >= 376)
    {
        start = evalRounds;
        end = evalRounds;
    }
    else
    {
        start = 1;
        end = 400;
    }
    map<bitset<256>, int, cmpBitset256> countingBox;
    int threadnum = 32;
    for (int i =start; i < end; i++)
    {
        countingBox.clear();
        int result = tinyjambuThreeEnumeration(cube, i, countingBox, threadnum);
        if (result == -1) cout << "Infeasible for round " << i << endl;
        else
        {
            if (countingBox.size() == 0) cout << "zero sum" << endl;
            cout << "FEASIBLE for round " << i << endl;
            cout << "The upperBound of round " << i << " is " << result << endl;
            
            cout << "*****************************" << endl;
            cout << "Final solution" << endl;
            cout << countingBox.size() << " solutions are found" << endl;
            map<bitset<256>, int, cmpBitset256> countingBox2;
            auto it = countingBox.begin();
            while (it != countingBox.end()) {
                bitset<256> tmp = (*it).first;
                for (int i = 96; i < 128; i++) {
                    if (cube[i - 96] == 1)
                        tmp[i] = 0;
                }
                countingBox2[tmp] += (*it).second;
                it++;
            }

            cout << "Odd" << endl;
            auto it2 = countingBox2.begin();
            while (it2 != countingBox2.end()) {
                if (((*it2).second % 2) == 1) {
                    cout << ((*it2).second % 2) << " | " << (*it2).second << "\t";
                    bitset<256> tmp = (*it2).first;
                    for (int i = 128; i < 256; i++) {
                        if ((tmp[i] == 1)) {
                            cout << "k" << (i - 128) << " ";
                        }
                    }
                    cout << endl;
                }
                it2++;
            }

            cout << "Even" << endl;
            it2 = countingBox2.begin();
            while (it2 != countingBox2.end()) {
                if (((*it2).second % 2) == 0) {
                    cout << ((*it2).second % 2) << " | " << (*it2).second << "\t";
                    bitset<256> tmp = (*it2).first;
                    for (int i = 128; i < 256; i++) {
                        if ((tmp[i] == 1)) {
                            cout << "k" << (i - 128) << " ";
                        }
                    }
                    cout << endl;
                }
                it2++;
            }
        }
    }
    /*int result = tinyjambuThreeEnumeration(cube, evalRounds, countingBox, threadnum);
    cout << "result: " << result << endl;*/

}



int main()
{
    int excludeCubeIndex, evalRounds;
    cout << "Choose the excluding nonce index(0-31)(else enter other value will include all cube): ";
    cin >> excludeCubeIndex;
    cout << "Choose the evaluation rounds (376++)(else enter other value will will do 1~400): ";
    cin >> evalRounds;
    string outname = "out_ExcludeCubeIndex_" + to_string(excludeCubeIndex)+"R"+to_string(evalRounds);
    ofstream out(outname.append(".txt"));
    
    streambuf* coutbuf = cout.rdbuf(); //save old buf
    cout.rdbuf(out.rdbuf()); //redirect cout to out.txt!

    attack(excludeCubeIndex, evalRounds);

    cout.rdbuf(coutbuf); //reset to standard output again

    cout << "Program ended!" << endl;
    
}