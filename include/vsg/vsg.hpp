#pragma once
//------------------------------------------------------------------------------
// A simple scene graph and virtual scene graph to support reactive rendering.
//
// Attempts to use C++ value semantics as defined by
// https://github.com/arximboldi/lager
//
// Uses immutable data structures from:
// https://github.com/arximboldi/immer
//
// Depends on the idea that a forest of const shared_ptrs have value semantics.
// (there are some caveats to the above, but generally it works)
//
// TODO: Needs to actually be a scene graph (include model matrices)
// TODO: Ensure that we can also have 2D (orthographic views)
//------------------------------------------------------------------------------
#include <any>
#include <memory>
#include <type_traits>
#include <utility>

#include <immer.h>

namespace vsg
{

// Not part of our public interface.
template<typename T>
using vector = immer::vector<T>;

// Not part of our public interface.
template<typename T>
using box = std::shared_ptr<const T>;

// Temporary function used to make the compiler happy.
// TODO: make this not usable somehow
template<typename ReturnT, typename ArgT>
auto updateGPU(ArgT const& /*unused*/) -> box<ReturnT>
{
  return box<ReturnT> {};
}

// TODO: make this "view" be dependent on a concept for providing a
// model/view/projection matrix?
template<typename View>
struct Context
{
  View view;

  explicit Context(View _view)
      : view {std::move(_view)}
  {
  }
};

struct Placeholder
{
};
template<typename View>
void draw(box<Context<View>> ctx,
          Placeholder const& placeHolder1,
          Placeholder const& placeHolder2);

template<typename View>
struct NodeI
{
  NodeI() = default;
  NodeI(const NodeI&) = default;
  NodeI(NodeI&&) = default;
  auto operator=(const NodeI&) -> NodeI& = default;
  auto operator=(NodeI&&) -> NodeI& = default;
  virtual ~NodeI() = default;

  virtual void draw(box<Context<View>> ctx) const = 0;
  [[nodiscard]] virtual auto marker() const -> std::any = 0;
  [[nodiscard]] virtual auto drawArgs() const -> std::any = 0;
  [[nodiscard]] virtual auto children() const -> vector<box<NodeI>> = 0;
  virtual void updateChildren(vector<box<NodeI>> const& newChildren) = 0;
  virtual void updateDrawArgs(std::any drawArgs) = 0;
};

template<typename DrawArgsT, typename ChangeMarkerT, typename View>
struct Node : public NodeI<View>
{
  DrawArgsT _drawArgs;
  using GPUDataT = decltype(updateGPU(_drawArgs));
  GPUDataT gpuData;
  ChangeMarkerT _changeMarker;

  // TODO: This makes the children completely generic, which means ONLY the
  // Context
  //       can be passed along, none of the DrawArgsT. :(
  //       Can we fix that?
  vector<box<NodeI<View>>> _children;

  Node(DrawArgsT args, ChangeMarkerT marker, vector<box<NodeI<View>>> children)
      : _drawArgs {args}
      , gpuData {updateGPU(args)}
      , _changeMarker {marker}
      , _children {std::move(children)}
  {
  }

  // Rule of 5.
  ~Node() override = default;
  Node(const Node&) = default;
  Node(Node&&) noexcept = default;
  auto operator=(const Node&) -> Node& = default;
  auto operator=(Node&&) noexcept -> Node& = default;

  // Provide the base interface elements for our types.
  [[nodiscard]] auto marker() const -> std::any override
  {
    return std::any(_changeMarker);
  }
  [[nodiscard]] auto drawArgs() const -> std::any override
  {
    return std::any(_drawArgs);
  }

  void draw(box<Context<View>> ctx) const override
  {
    using dggs::draw::draw;
    draw(ctx, gpuData, _drawArgs);
    immer::for_each(_children,
                    [&](const box<NodeI<View>>& node) { node->draw(ctx); });
  }

  [[nodiscard]] auto children() const -> vector<box<NodeI<View>>> override
  {
    return _children;
  }

  void updateChildren(vector<box<NodeI<View>>> const& newChildren) override
  {
    _children = newChildren;
  }

  void updateDrawArgs(std::any newArgs) override
  {
    if (newArgs.type() != typeid(_drawArgs)) {
      return;
    }
    _drawArgs = std::any_cast<DrawArgsT>(newArgs);
  }
};

struct vNodeI
{
  vNodeI() = default;
  vNodeI(const vNodeI&) = default;
  vNodeI(vNodeI&&) = default;
  auto operator=(const vNodeI&) -> vNodeI& = default;
  auto operator=(vNodeI&&) -> vNodeI& = default;
  virtual ~vNodeI() = default;

  [[nodiscard]] virtual auto marker() const -> std::any = 0;
  [[nodiscard]] virtual auto drawArgs() const -> std::any = 0;
  [[nodiscard]] virtual auto children() const -> vector<box<vNodeI>> = 0;
  [[nodiscard]] virtual auto changeMarkerEquals(NodeI const& other) const
      -> bool = 0;
  [[nodiscard]] virtual auto node() const -> box<NodeI> = 0;
};

template<typename DrawArgsT, typename ChangeMarkerT>
struct vNode : public vNodeI
{
  DrawArgsT _drawArgs;
  ChangeMarkerT _changeMarker;
  vector<box<vNodeI>> _children;

  vNode(DrawArgsT args, ChangeMarkerT marker, vector<box<vNodeI>> children)
      : _drawArgs {args}
      , _changeMarker {marker}
      , _children {std::move(children)}
  {
  }

  // Rule of 5.
  ~vNode() override = default;
  vNode(const vNode&) = default;
  vNode(vNode&&) noexcept = default;
  auto operator=(const vNode&) -> vNode& = default;
  auto operator=(vNode&&) noexcept -> vNode& = default;

  // Provide the base interface elements for our types.
  [[nodiscard]] auto marker() const -> std::any override
  {
    return std::any(_changeMarker);
  }

  [[nodiscard]] auto drawArgs() const -> std::any override
  {
    return std::any(_drawArgs);
  }

  [[nodiscard]] auto children() const -> vector<box<vNodeI>> override
  {
    return _children;
  }
  [[nodiscard]] auto changeMarkerEquals(NodeI const& other) const
      -> bool override
  {
    auto otherChangeMarker = other.marker();
    if (otherChangeMarker.type() != typeid(_changeMarker)) {
      return false;
    }
    return std::any_cast<ChangeMarkerT>(otherChangeMarker) == _changeMarker;
  }

  [[nodiscard]] auto node() const -> box<NodeI> override
  {
    vector<box<NodeI>> childrenNodes;
    immer::for_each(_children,
                    [&](const box<vNodeI>& child) {
                      childrenNodes = childrenNodes.push_back(child->node());
                    });
    return std::make_shared<Node<DrawArgsT, ChangeMarkerT>>(
        _drawArgs, _changeMarker, childrenNodes);
  }
};

auto updateGraph(box<NodeI> graph, box<vNodeI> const& vGraph) -> box<NodeI>
{
  if (!graph) {
    // If the current graph is empty, create it from scratch.
    return vGraph->node();
  }

  // Else try and update things.
  if (
      // if the changeMarker did not change
      vGraph->changeMarkerEquals(*graph) &&
      // and the type of the node did not change
      vGraph->drawArgs().type() == graph->drawArgs().type()

  )
  {
    // This node will remain the same, but update its children
    std::size_t index = 0;

    auto oldChildren = graph->children();
    auto vChildren = vGraph->children();
    immer::vector<box<NodeI>> newChildren;

    while (index < vChildren.size()) {
      if (index < oldChildren.size()) {
        // Patch the children together
        newChildren = newChildren.push_back(
            updateGraph(oldChildren[index], vChildren[index]));
      } else {
        // Completely new child.
        newChildren = newChildren.push_back(vChildren[index]->node());
      }
      ++index;
    }
    graph->updateDrawArgs(vGraph->drawArgs());
    graph->updateChildren(newChildren);
    return graph;
  }
  // Create the entire thing from this point onwards.
  return vGraph->node();
}

//--------------------------------------------------------------------------
// In HTML we'd have loads of nodes to use as "containers" here, we don't
// So we'll make a simple one.
struct Fragment
{
};
struct FragmentGPUData
{
};

auto updateGPU(Fragment frag) -> std::shared_ptr<FragmentGPUData>
{
  return std::make_shared<FragmentGPUData>();
}
void draw(std::shared_ptr<draw::Context> /*unused*/,
          std::shared_ptr<FragmentGPUData> /*unused*/,
          Fragment const& /*unused*/)
{
}

//--------------------------------------------------------------------------
// Helper functions that allow various options
template<typename DrawArgsT, typename ChangeMarkerT>
auto n(DrawArgsT val, ChangeMarkerT marker, vector<box<vNodeI>> children)
    -> box<vNodeI>
{
  return std::make_shared<vNode<DrawArgsT, ChangeMarkerT>>(
      val, marker, children);
}

// Specialization to allow for use without a marker.
struct NoChangeMarker
{
};
auto operator==(NoChangeMarker const& lhs, NoChangeMarker const& rhs) -> bool
{
  return true;
}

template<typename DrawArgsT>
auto n(DrawArgsT val, vector<box<vNodeI>> children) -> box<vNodeI>
{
  return n(val, NoChangeMarker {}, children);
}

template<typename DrawArgsT, typename ChangeMarkerT>
auto n(DrawArgsT val, ChangeMarkerT marker) -> box<vNodeI>
{
  return n(val, marker, {});
}

template<typename DrawArgsT>
auto n(DrawArgsT val) -> box<vNodeI>
{
  return n(val, NoChangeMarker {}, {});
}

template<typename ChangeMarkerT>
auto f(ChangeMarkerT marker, vector<box<vNodeI>> children) -> box<vNodeI>
{
  return n(Fragment {}, marker, children);
}

auto f(vector<box<vNodeI>> children) -> box<vNodeI>
{
  return n(Fragment {}, NoChangeMarker {}, std::move(children));
}
auto empty() -> box<vNodeI>
{
  return n(Fragment {}, NoChangeMarker {}, {});
}

// A helper to define values that aren't used until needed.
template<typename T>
class LazyVal
{
protected:
  using GenT = std::function<T()>;
  GenT generator;

public:
  explicit LazyVal(T val)
      : generator {[val] { return val; }}
  {
  }

  explicit LazyVal(GenT gen)
      : generator {[gen]() { return gen(); }}
  {
  }

  explicit operator T() { return generator(); }

  auto operator()() -> T { return generator(); }
};
}  // namespace vsg

using namespace dggs;

void draw::draw(std::shared_ptr<draw::Context> /*unused*/,
                std::shared_ptr<FragmentGPUData> /*unused*/,
                Fragment const& /*unused*/)
{
}
