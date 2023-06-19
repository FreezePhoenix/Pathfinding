#include <functional>
#include <vector>
#include <algorithm>

template<typename K, typename T, typename C = std::less<K>>
	requires std::predicate<C, K, K>
class PriorityVectorQueue {
	std::vector<std::pair<K, T>> heap;
public:
	constexpr void push(const K& key, const T& elem) {
		heap.emplace_back(key, elem);
		std::push_heap(heap.begin(), heap.end(), [](const auto& left, const auto& right) {
			return C()(left.first, right.first);
		});
	};
	constexpr void push(K&& key, T&& elem) {
		heap.emplace_back(std::move(key), std::move(elem));
		std::push_heap(heap.begin(), heap.end(), [](const auto& left, const auto& right) {
			return C()(left.first, right.first);
		});
	};
	
	template<typename... Args>
		requires std::constructible_from<T, Args...>
	constexpr void emplace(const K& key, Args&&... args) {
		heap.emplace_back(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(args...));
		std::push_heap(heap.begin(), heap.end(), [](const auto& left, const auto& right) {
			return C()(left.first, right.first);
		});
	}
	
	constexpr const T& top() const {
		return heap.front().second;
	}
	
	constexpr void pop() {
		std::pop_heap(heap.begin(), heap.end(), [](auto& left, auto& right) {
			return C()(left.first, right.first);
		});
		heap.pop_back();
	}

	friend void swap(PriorityVectorQueue<K, T, C>& lhs, PriorityVectorQueue<K, T, C>& rhs) {
		std::swap(lhs.heap, rhs.heap);
	}

	template<typename I = std::equal_to<T>>
		requires std::predicate<I, T, T>
	constexpr void raise_priority(const K& new_key, const T& value) {
		// TODO: Is there a better way to do this?
		auto it = std::find_if(heap.begin(), heap.end(), [&value](const auto& entry) {
			return I()(entry.second, value);
		});

		if (it != heap.end()) {
			it->first = new_key;
			std::push_heap(heap.begin(), it, [](const auto& left, const auto& right) {
				return C()(left.first, right.first);
			});
		}
	}

	template<typename I = std::equal_to<T>>
		requires std::predicate<I, T, T>
	constexpr void raise_priority(K&& new_key, T&& value) {
		// TODO: Is there a better way to do this?
		auto it = std::find_if(heap.begin(), heap.end(), [&value](const auto& entry) {
			return I()(entry.second, value);
		});

		if (it != heap.end()) {
			it->first = new_key;
			std::push_heap(heap.begin(), it, [](const auto& left, const auto& right) {
				return C()(left.first, right.first);
			});
		}
	}

	template<typename I, typename... Args>
		requires std::predicate<I, T, Args...> && std::constructible_from<T, Args...>
	constexpr void raise_priority(const K& new_key, Args&&... args) {
		// TODO: Is there a better way to do this?
		auto it = std::find_if(heap.begin(), heap.end(), [&args...](const auto& entry) {
			return I()(entry.second, std::forward<Args>(args)...);
		});

		if (it != heap.end()) {
			it->first = new_key;
			std::push_heap(heap.begin(), it, [](const auto& left, const auto& right) {
				return C()(left.first, right.first);
			});
		}
	}

	template<typename I, typename... Args>
		requires std::predicate<I, T, Args...> && std::constructible_from<T, Args...>
	constexpr void raise_priority(K&& new_key, Args&&... args) {
		// TODO: Is there a better way to do this?
		auto it = std::find_if(heap.begin(), heap.end(), [&args...](const auto& entry) {
			return I()(entry.second, std::forward<Args>(args)...);
		});

		if (it != heap.end()) {
			it->first = std::move(new_key);
			std::push_heap(heap.begin(), it, [](const auto& left, const auto& right) {
				return C()(left.first, right.first);
			});
		}
	}

	template<typename I, typename... Args>
		requires std::predicate<I, T, T> && std::constructible_from<T, Args...>
	constexpr void raise_priority(const K& new_key, Args&&... args) {
		raise_priority<I>(new_key, { std::forward<Args>(args)... });
	}

	template<typename I, typename... Args>
		requires std::predicate<I, T, T> && std::constructible_from<T, Args...>
	constexpr void raise_priority(K&& new_key, Args&&... args) {
		raise_priority<I>(std::move(new_key), { std::forward<Args>(args)... });
	}

	constexpr bool empty() const {
		return heap.empty();
	}

	constexpr size_t size() const {
		return heap.size();
	}
};

namespace std {
    template<typename K, typename T, typename C = std::less<K>>
		requires std::predicate<C, K, K>
    void swap(PriorityVectorQueue<K, T, C>& lhs, PriorityVectorQueue<K, T, C>& rhs) {
        swap(lhs, rhs);
    }
}