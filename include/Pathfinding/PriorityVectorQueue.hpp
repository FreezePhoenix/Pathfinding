#include <functional>
#include <vector>
#include <algorithm>

template<typename K, typename T, typename C = std::less<K>>
	requires std::predicate<C, K, K>
class PriorityVectorQueue {
	std::vector<std::pair<K, T>> heap;
private:
	inline void bubble(size_t index) {
		while (index > 0 && C()(heap[(index - 1) / 2].first, heap[index].first)) {
			std::swap(heap[index], heap[(index - 1) / 2]);
			index = (index - 1) / 2;
		}
	}
	
	inline void sink(size_t index) {
        size_t size = heap.size();
        while (2 * index + 1 < size) {
            size_t child = 2 * index + 1;

            if (child + 1 < size && C()(heap[child].first, heap[child + 1].first)) {
                ++child;
            }

            if (C()(heap[index].first, heap[child].first)) {
                std::swap(heap[index], heap[child]);
                index = child;
            } else {
                break;
            }
        }
	}
public:
	inline PriorityVectorQueue() : heap() {

	};
	/**
	 * @brief Insert a new element into the PriorityQueue
	*/
	inline void insert(K key, T elem) {
		heap.emplace_back(key, elem);
		bubble(heap.size() - 1);
		// std::push_heap(heap.begin(), heap.end(), [](const auto& left, const auto& right) {
		// 	return C()(left.first, right.first);
		// });
	};
	/**
	 * @brief Construct a new element from the given arguments in the PriorityQueue
	*/
	template<typename... Args>
		requires std::constructible_from<T, Args...>
	inline void emplace(K key, Args&&... args) {
		heap.emplace_back(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(args...));
		bubble(heap.size() - 1);
		// std::push_heap(heap.begin(), heap.end(), [](const auto& left, const auto& right) {
		// 	return C()(left.first, right.first);
		// });
	}

	/**
	 * @brief Access the top element from the PriorityQueue
	*/
	inline const T& top() const {
		return heap.front().second;
	}

	inline T remove() {
		T result = std::exchange(heap.front(), heap.back()).second;
		heap.pop_back();
		sink(0);
		return result;
	}

	/**
	 * @brief Remove the top element from the PriorityQueue
	*/
	inline void pop() {
		heap.front() = heap.back();
		heap.pop_back();
		sink(0);
		// std::pop_heap(heap.begin(), heap.end(), [](auto& left, auto& right) {
		// 	return C()(left.first, right.first);
		// });
		// heap.pop_back();
	}

	/**
	 * @brief Re-key a previously inserted key into the 
	*/
	template<typename I = std::equal_to<T>>
		requires std::predicate<I, T, T>
	inline void raise_priority(K new_key, T value) {
		// TODO: Is there a better way to do this?
		auto it = std::find_if(heap.cbegin(), heap.cend(), [&value](const auto& entry) {
			return I()(entry.second, value);
		});

		if (it != heap.cend()) {
			size_t index = std::distance(heap.cbegin(), it);
			heap[index].first = new_key;
			
			bubble(index);
		}
	}

	template<typename I, typename... Args>
		requires std::predicate<I, T, Args...> && std::constructible_from<T, Args...>
	inline void raise_priority(K new_key, Args&&... args) {
		// TODO: Is there a better way to do this?
		auto it = std::find_if(heap.cbegin(), heap.cend(), [&args...](const auto& entry) {
			return I()(entry.second, std::forward<Args>(args)...);
		});

		if (it != heap.cend()) {
			size_t index = std::distance(heap.cbegin(), it);
			heap[index].first = new_key;
			
			bubble(index);
		}
	}

	template<typename I, typename... Args>
		requires std::predicate<I, T, T> && std::constructible_from<T, Args...>
	inline void raise_priority(K new_key, Args&&... args) {
		raise_priority<I>(new_key, { std::forward<Args>(args)... });
	}
	
	inline bool empty() const {
		return heap.empty();
	}
};