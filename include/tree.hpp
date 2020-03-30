//
// Created by Raghav Anand on 3/25/20.
//

#ifndef MPLAMBDA_TREE_HPP
#define MPLAMBDA_TREE_HPP

#include <vector>
#include <queue>

namespace mpl {
    template<class T>
    class Tree {
        // TODO: figure out the correct way of writing this class later

    private:

        T data;
        Tree<T>* parent;
        std::vector< Tree<T>* > children;

    public:
        bool changed = false;

        Tree(T data_, Tree<T>* parent_) : parent(parent_), data(data_) { }
        explicit Tree(T data_) : parent(nullptr), data(data_) { }

        const T& getData() const {
            return data;
        }

        void setData(T& data_) {
            data = data_;
        }

        void addChild(T& data_) {
            children.push_back(new Tree<T>(data_, this));
        }

        const std::vector<Tree<T>*>& getChildren() const {
            return children;
        }

        ~Tree() {
            for(Tree<T>* childNode : children) delete childNode;
        }

//        void removeChild(const size_t& indx) {
//            children.erase(children.begin()+indx);
//        }
//
//
//        Tree<T>* findChild(const T& data) const {
//            for(int i=0; i<children.size(); i++)
//                if(children[i]->getData() == data)
//                    return children[i];
//            return NULL;
//        }
//
//
//        Tree<T>* getChild(const size_t& indx) const {
//            return children[indx];
//        }


        Tree<T>* getParent() const {
            return parent;
        }

        int getNumChildren() const {
            return children.size();
        }
    };


}

template <class Node>
std::ostream &operator<<(std::ostream &os, const mpl::Tree<Node> &tree) {
    int curr_depth = 0;
    typedef std::pair<mpl::Tree<Node>*, int> tree_depth_pair;
    os << tree.getData();
    std::queue<tree_depth_pair> print_helper;
    for (auto c : tree.getChildren()) {
        print_helper.push(tree_depth_pair(c, curr_depth + 1));
    }
    while (!print_helper.empty()) {
        auto& [node, depth] = print_helper.front();
        print_helper.pop();
        if (depth != curr_depth) {
            os << '\n';
            curr_depth = depth;
        }
        os << node->getData() << " ";
        for (auto c : node->getChildren()) {
            print_helper.push(tree_depth_pair(c, depth + 1));
        }
    }
    return os;
}

#endif //MPLAMBDA_TREE_HPP
