#include <functional>
#include <vector>
#include <algorithm>

// TODO: Figure out why this doesn't actually work
template<typename T, typename C = std::less<T>, typename I = std::equal_to<T>>
class PriorityFibonacciQueue {
	class Node {
		public:
            Node* parent;
			Node* prev;
			Node* next;
			Node* child;
			size_t rank;
			T value;
			bool marked;
			inline Node(Node* prev, Node* next, const T& value) {
                this->parent = nullptr;
				this->prev = prev;
				this->next = next;
				this->child = nullptr;
				this->rank = 0;
				this->value = value;
				this->marked = 0;
			}
            template<typename... Args>
            inline Node(Node* prev, Node* next, Args&&... args): value(std::forward<Args>(args)...) {
                this->parent = nullptr;
				this->prev = prev;
				this->next = next;
				this->child = nullptr;
				this->rank = 0;
				this->marked = 0;
			}
			inline void insert_child(Node* other) {
                this->rank++;
                other->parent = this;
				if (child == nullptr) {
					other->next = other;
					other->prev = other;
					this->child = other;
				} else {
					other->prev = this->child;
					other->next = this->child->next;
					this->child->next->prev = other;
					this->child->next = other;
				}
			}
            template<typename... Args>
            inline Node(Args&&... args) : Node(this, this, std::forward<Args>(args)...) {
				
			}
			inline Node(const T& value) : Node(this, this, value) {
				
			}
	};
	Node* top_node;
    std::unordered_map<T, Node*> value_to_node;
    size_t max_rank;
    inline void remove(Node* node) {
        if(node->next != node) {
            node->next->prev = node->prev;
            node->prev->next = node->next;
        }
    }
	inline Node* link(Node* left, Node* right) {
		if (C()(left->value, right->value)) { // right has greater priority than left
			remove(left);
            right->insert_child(left);
            return right;
		} else {
            remove(right);
			left->insert_child(right);
            return left;
		}
	}
    inline void cut(Node* node) {
        node->parent = nullptr;
        node->prev = this->top_node;
        node->next = this->top_node->next;
        this->top_node->next->prev = node;
        this->top_node->next = node;
        if (C()(top_node->value, node->value)) {
            this->top_node = node;
        }
    }
    
    inline void consolidate() {
        if(this->top_node == nullptr) {
            return;
        }
        std::vector<Node*> trees = std::vector<Node*>();
        Node* current = top_node;
        do {
            while(current->rank < trees.size() && trees[current->rank] != nullptr) {
                Node* other = trees[current->rank];
                trees[current->rank] = nullptr;
                if (C()(current->value, other->value)) {
                    std::swap(current, other);
                }
                link(other, current);
            }
            if(current->rank >= trees.size()) {
                trees.reserve(current->rank + 1);
            }
            trees[current->rank] = current;
            current = current->next;
        } while (current != top_node);
    }
    inline void fix_top() {
        if(this->top_node == nullptr) {
            return;
        }
		Node* new_top = this->top_node;
		Node* current = this->top_node;
        do {
			if (C()(new_top->value, current->value)) { // current has greater priority than new_top
				new_top = current;
			}
			current = current->next;
        } while(current != this->top_node);
		this->top_node = new_top;
	}
    public:
    
	
    template<typename... Args>
	inline void raise_priority(Args&&... args) {
		T new_key(std::forward<Args>(args)...);
		Node* target = value_to_node[new_key];
        Node* parent = target->parent;
        assert(C()(target->value, new_key));
        target->value = new_key;
        if(parent != nullptr) {
            if(C()(parent->value, target->value)) {
                cut(target);
                if(parent->marked) {
                    cut(parent);
                    parent->marked = false;
                } else {
                    parent->marked = true;
                }
            }
        } else {
            if (C()(top_node->value, target->value)) {
                this->top_node = target;
            }
        }
    }
    inline void raise_priority(T new_key) {
        Node* target = value_to_node[new_key];
        Node* parent = target->parent;
        assert(C()(target->value, new_key));
        target->value = new_key;
        if(parent != nullptr) {
            if(C()(parent->value, target->value)) {
                cut(target);
                if(parent->marked) {
                    cut(parent);
                    parent->marked = false;
                } else {
                    parent->marked = true;
                }
            }
        } else {
            if (C()(top_node->value, target->value)) {
                this->top_node = target;
            }
        }
    }
	inline void insert(const T& elem) {
		if (top_node == nullptr) {
			top_node = new Node(elem);
            this->value_to_node.emplace(elem, top_node);
		} else {
			Node* temp = new Node(this->top_node, this->top_node->next, elem);
            this->value_to_node.emplace(elem, temp);
			this->top_node->next->prev = temp;
			this->top_node->next = temp;
			if (C()(top_node->value, temp->value)) {
				this->top_node = temp;
			}
		}
	};

    template<typename... Args>
    inline void emplace(Args&&... args) {
		if (top_node == nullptr) {
			top_node = new Node(std::forward<Args>(args)...);
            this->value_to_node.emplace(top_node->value, top_node);
		} else {
			Node* temp = new Node(this->top_node, this->top_node->next, std::forward<Args>(args)...);
            this->value_to_node.emplace(temp->value, temp);
			this->top_node->next->prev = temp;
			this->top_node->next = temp;
			if (C()(top_node->value, temp->value)) {
				this->top_node = temp;
			}
		}
	};
	
    inline const T& top() {
        return top_node->value;
    }
	inline void pop() {
		// Scenario: Linked list A <-> B <-> C <-> A
		//			 Linked list 1 <-> 2 <-> 3 <-> 1
		// this->top == B
		// this->top->child == 2
		// Goal: Linked list A <-> 2 <-> 3 <-> 1 <-> C <-> A
        Node* new_top;
        if(this->top_node->child != nullptr) {
            
            Node* current = this->top_node->child;
            do {
                current->parent = nullptr;
                current = current->next;
            } while(current != this->top_node->child);
            this->top_node->prev->next = this->top_node->child;
            this->top_node->next->prev = this->top_node->child->prev;
            this->top_node->child->prev->next = this->top_node->next;
            this->top_node->child->prev = this->top_node->prev;
            new_top = this->top_node->child;
        } else {
            if(this->top_node->next == this->top_node) {
                new_top = nullptr;
            } else {
                this->top_node->next->prev = this->top_node->prev;
                this->top_node->prev->next = this->top_node->next;
                new_top = this->top_node->next;
            }
        }
        this->value_to_node.erase(this->top_node->value);
        // delete this->top_node;
        this->top_node = new_top;
		fix_top();
        consolidate();
	}
    inline bool empty() {
        return top_node == nullptr;
    }
};