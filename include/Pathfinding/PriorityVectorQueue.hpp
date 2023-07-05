#include <functional>
#include <vector>
#include <algorithm>
#include <concepts>

template<typename K, typename T, typename C = std::less<K>, typename I = std::equal_to<T>>
	requires std::predicate<C, K, K> && std::predicate<I, T, T>
class PriorityVectorQueue {
	std::vector<std::pair<K, T>> heap;
	[[no_unique_address]]
	struct value_comp {
		[[no_unique_address]]
		C comparator;
		constexpr value_comp(const C& c) : comparator(c) {
		};
		constexpr bool operator()(const std::pair<K, T>& left, const std::pair<K, T>& right) const {
			return comparator(left.first, right.first);
		}
		constexpr bool operator()(const K& left, const K& right) const {
			return comparator(left, right);
		}
	} value_compare;
	[[no_unique_address]]
	struct identity {
		[[no_unique_address]]
		I predicate;
		constexpr identity(const I& i) : predicate(i) {

		};
		constexpr bool operator()(const std::pair<K, T>& left, const T& right) const {
			return predicate(left.second, right);
		}
		template<typename TT>
			requires std::predicate<I, T, TT>
		constexpr bool operator()(const std::pair<K, T>& left, TT&& right) const {
			return predicate(left.second, std::forward<TT>(right));
		}
	} value_ident;
public:
	constexpr PriorityVectorQueue() : value_compare(C()), value_ident(I()) {
	}
	constexpr PriorityVectorQueue(const C& c) : value_compare(c), value_ident(I()) {
	};
	constexpr PriorityVectorQueue(const I& i, const C& c = C()) : value_compare(c), value_ident(i) {
	};
	constexpr void push(const std::pair<K, T>& entry) {
		heap.push_back(entry);
		std::push_heap(heap.begin(), heap.end(), value_compare);
	};
	constexpr void push(std::pair<K, T>&& entry) {
		heap.push_back(std::move(entry));
		std::push_heap(heap.begin(), heap.end(), value_compare);
	};
	
	template<typename... Args>
		requires std::constructible_from<std::pair<K, T>, Args...>
	constexpr void emplace(Args&&... args) {
		heap.emplace_back(std::forward<Args>(args)...);
		std::push_heap(heap.begin(), heap.end(), value_compare);
	}
	
	constexpr const T& top() const noexcept {
		return heap.front().second;
	}
	
	constexpr void pop() {
		std::pop_heap(heap.begin(), heap.end(), value_compare);
		heap.pop_back();
	}

	constexpr friend void swap(PriorityVectorQueue<K, T, C, I>& lhs, PriorityVectorQueue<K, T, C, I>& rhs) {
		std::swap(lhs.heap, rhs.heap);
		std::swap(lhs.value_compare, rhs.value_compare);
		std::swap(lhs.value_ident, rhs.value_ident);
	}
	constexpr void raise_priority(const K& new_key, const T& value) {
		// TODO: Is there a better way to do this?
		auto it = std::find_if(heap.begin(), heap.end(), std::bind(value_ident, std::placeholders::_1, std::cref(value)));

		if (it != heap.end()) { [[likely]]
			it->first = new_key;
			std::push_heap(heap.begin(), it, value_compare);
		}
	}
	template<typename TT>
		requires std::predicate<I, T, TT>
	constexpr void raise_priority(const K& new_key, TT&& value) {
		// TODO: Is there a better way to do this?
		auto it = std::find_if(heap.begin(), heap.end(), std::bind(value_ident, std::placeholders::_1, std::cref(value)));

		if (it != heap.end()) { [[likely]]
			it->first = new_key;
			std::push_heap(heap.begin(), it, value_compare);
		}
	}

	constexpr bool empty() const noexcept {
		return heap.empty();
	}

	constexpr size_t size() const noexcept {
		return heap.size();
	}
};

namespace std {
    template<typename K, typename T, typename C = std::less<K>, typename I = std::equal_to<T>>
		requires std::predicate<C, K, K> && std::predicate<I, T, T>
    constexpr void swap(PriorityVectorQueue<K, T, C>& lhs, PriorityVectorQueue<K, T, C>& rhs) {
        swap(lhs, rhs);
    }
}