#ifndef DSTAR_LCT_H
#define DSTAR_LCT_H
#include "dstar_main/dstar_main.h"
class dstarNode::LinkCutTree{
    public:
        void reverse(Nodeptr now){now->reversed ^= 1;}
        bool get_son(Nodeptr x){return x->father->son[1] == x;}
        bool is_root(Nodeptr x){return x->father == nullptr || (x->father->son[0] != x && x->father->son[1] != x);}
        void rotate(Nodeptr x){
            Nodeptr f = x->father, ff = f->father;
            bool son = get_son(x);
            if(!is_root(f)) ff->son[get_son(f)] = x; x -> father = ff;
            f -> son[son] = x -> son[son^1]; if(x->son[son^1] != nullptr) x->son[son^1]->father = f;
            x -> son[son^1] = f; f -> father = x;
        }
        void pushdown(Nodeptr now){
            if(now->reversed){
                now->reversed = false;
                if(now->son[0] != nullptr) now->son[0]->reversed ^= 1;
                if(now->son[1] != nullptr) now->son[1]->reversed ^= 1;
                std::swap(now->son[0], now->son[1]);
            }
            return;
        }
        void splay(Nodeptr now){
            std::stack<Nodeptr> s;
            Nodeptr temp = now;
            while(!is_root(temp)){
                s.push(temp);
                temp = temp->father;
            }
            s.push(temp);
            while(!s.empty()){
                pushdown(s.top());
                s.pop();
            }
            while(!is_root(now)){
                if(!is_root(now -> father)){
                    if(get_son(now->father) == get_son(now)) rotate(now->father);
                    else rotate(now);
                }
                rotate(now);
            }
        }
        void access(Nodeptr now){
            Nodeptr last = nullptr;
            for(Nodeptr x = now; x != nullptr; x = x->father){
                splay(x);
                x->son[1] = last;
                last = x;
            }
        }
        void makeroot(Nodeptr now){
            access(now);
            splay(now);
            reverse(now);
        }
        Nodeptr find_root(Nodeptr now){
            access(now);
            splay(now);
            while(now->son[0] != nullptr) now = now->son[0];
            return now;
        }
        void link(Nodeptr x, Nodeptr y){
            if(find_root(x) == find_root(y))return ;
            makeroot(x);
            makeroot(y);
            x->father = y;
        }
        void del(Nodeptr x, Nodeptr y){
            if(x == nullptr || y == nullptr)return;
            makeroot(x);
            access(y);
            splay(y);
            if(y->son[0] == x && x->son[1] == nullptr && x->son[0] == nullptr){ 
                y->son[0] = nullptr;
                x->father = nullptr;
            }
        }
};
#endif