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

#include <immer/algorithm.hpp>
#include <immer/vector.hpp>

namespace vsg {

// Not part of our public interface.
template<typename T>
using Vector = immer::vector<T>;

// Not part of our public interface.
template<typename T>
using Box = std::shared_ptr<T>;

// Temporary function used to make the compiler happy.
// TODO: make this not usable somehow
template<typename ReturnT, typename ArgT>
auto updateGPU(ArgT const & /*unused*/) -> Box<ReturnT> {
  return Box<ReturnT>{};
}

struct Placeholder {};
template<typename ContextT>
void draw(
  Box<ContextT> const &ctx,
  Placeholder const &placeHolder1,
  Placeholder const &placeHolder2
);

// This concept determines that a particule T can be drawn
template<typename ContextT, typename ArgsT, typename GPUDataT>
concept DrawableNode = requires(ContextT ctx, ArgsT args, GPUDataT data) {
                         { updateGPU(args) } -> std::convertible_to<GPUDataT>;
                         { draw(ctx, data, args) } -> std::convertible_to<void>;
                       };

// This concept determines that a particule T can be used to update the context
template<typename ContextT, typename ArgsT>
concept ContextNode = requires(ContextT ctx, ArgsT args) {
                        { updateGPU(ctx, args) } -> std::convertible_to<ContextT>;
                      };

//--------------------------------------------------------------------------------------------------
// The core API that nodes must implement.
template<typename ContextT>
struct NodeI {
  NodeI()                                                          = default;
  NodeI(NodeI<ContextT> const &)                                   = default;
  NodeI(NodeI<ContextT> &&) noexcept                               = default;
  auto operator=(NodeI<ContextT> const &) -> NodeI<ContextT>     & = default;
  auto operator=(NodeI<ContextT> &&) noexcept -> NodeI<ContextT> & = default;
  virtual ~NodeI()                                                 = default;

  virtual void draw(Box<ContextT> const &ctx) const                           = 0;
  [[nodiscard]] virtual auto markerAsAny() const -> std::any                  = 0;
  [[nodiscard]] virtual auto drawArgsAsAny() const -> std::any                = 0;
  [[nodiscard]] virtual auto children() const -> Vector<Box<NodeI<ContextT>>> = 0;
};

//------------------------------------------------------------------------------
// Nodes are the concrete elements of the scene graph that will be rendered.
//
// They hold the original args used to create them as well as the gpu data created
// when they were uploaded to the GPU and the change marker that was specified
// when they were created. They also hold a set of children nodes that forms the tree.
template<typename ContextT, typename DrawArgsT, typename ChangeMarkerT>
struct Node : public NodeI<ContextT> {
  DrawArgsT drawArgs;
  using GPUData = decltype(updateGPU(drawArgs));
  GPUData gpuData;
  ChangeMarkerT changeMarker;

  // TODO: This makes the children completely generic, which means ONLY the
  // Context
  //       can be passed along, none of the draw_args_t. :(
  //       Can we fix that?
  Vector<Box<NodeI<ContextT>>> children;

  Node(DrawArgsT args, ChangeMarkerT marker, Vector<Box<NodeI<ContextT>>> children)
    : drawArgs{args}
    , gpuData{updateGPU(args)}
    , changeMarker{marker}
    , children{std::move(children)} {}

  // Rule of 5.
  ~Node() override                           = default;
  Node(const Node &)                         = default;
  Node(Node &&) noexcept                     = default;
  auto operator=(const Node &) -> Node     & = default;
  auto operator=(Node &&) noexcept -> Node & = default;

  // Provide the base interface elements for our types.
  [[nodiscard]] auto markerasAny() const -> std::any override { return std::any(changeMarker); }
  [[nodiscard]] auto drawArgsAsAny() const -> std::any override { return std::any(drawArgs); }

  void draw(Box<ContextT> const &ctx) const override {
    using vsg::draw;
    draw(ctx, gpuData, drawArgs);
    immer::for_each(children, [&](const Box<NodeI<ContextT>> &curNode) { curNode->draw(ctx); });
  }
  void updateDrawArgs(std::any newArgs) override {
    if (newArgs.type() != typeid(drawArgs)) { return; }
    drawArgs = std::any_cast<DrawArgsT>(newArgs);
  }
};

//------------------------------------------------------------------------------
// Virtual nodes represent a potential graph element.
//
// They should be cheap to create and copy, and can be compared to a node in
// order to determine if they will create a different node.
// They only hold the args and change markerk
template<typename ContextT>
struct VirtualNodeI {
  VirtualNodeI()                                                                 = default;
  VirtualNodeI(const VirtualNodeI<ContextT> &)                                   = default;
  VirtualNodeI(VirtualNodeI<ContextT> &&) noexcept                               = default;
  auto operator=(const VirtualNodeI<ContextT> &) -> VirtualNodeI<ContextT>     & = default;
  auto operator=(VirtualNodeI<ContextT> &&) noexcept -> VirtualNodeI<ContextT> & = default;
  virtual ~VirtualNodeI()                                                        = default;

  [[nodiscard]] virtual auto markerAsAny() const -> std::any                                = 0;
  [[nodiscard]] virtual auto drawArgsAsAny() const -> std::any                              = 0;
  [[nodiscard]] virtual auto children() const -> Vector<Box<VirtualNodeI<ContextT>>>        = 0;
  [[nodiscard]] virtual auto changeMarkerEquals(NodeI<ContextT> const &other) const -> bool = 0;
  [[nodiscard]] virtual auto createNode() const -> Box<NodeI<ContextT>>                     = 0;
};

template<typename ContextT, typename DrawArgsT, typename ChangeMarkerT>
struct VirtualNode : public VirtualNodeI<ContextT> {
  DrawArgsT drawArgs;
  ChangeMarkerT changeMarker;
  Vector<Box<VirtualNodeI<ContextT>>> children;

  VirtualNode(DrawArgsT args, ChangeMarkerT marker, Vector<Box<VirtualNodeI<ContextT>>> children)
    : drawArgs{args}
    , changeMarker{marker}
    , children{std::move(children)} {}

  // Rule of 5.
  ~VirtualNode() override                                  = default;
  VirtualNode(const VirtualNode &)                         = default;
  VirtualNode(VirtualNode &&) noexcept                     = default;
  auto operator=(const VirtualNode &) -> VirtualNode     & = default;
  auto operator=(VirtualNode &&) noexcept -> VirtualNode & = default;

  // Provide the base interface elements for our types.
  [[nodiscard]] auto markerAsAny() const -> std::any override { return std::any(changeMarker); }

  [[nodiscard]] auto drawArgsAsAny() const -> std::any override { return std::any(drawArgs); }

  [[nodiscard]] auto changeMarkerEquals(NodeI<ContextT> const &other) const -> bool override {
    auto otherChangeMarker = other.marker();
    if (otherChangeMarker.type() != typeid(changeMarker)) { return false; }
    return std::any_cast<ChangeMarkerT>(otherChangeMarker) == changeMarker;
  }

  [[nodiscard]] auto createNode() const -> Box<NodeI<ContextT>> override {
    Vector<Box<NodeI<ContextT>>> childrenNodes;
    immer::for_each(children, [&](const Box<VirtualNodeI<ContextT>> &child) {
      childrenNodes = childrenNodes.push_back(child->createNode());
    });
    return std::make_shared<vsg::Node<ContextT, DrawArgsT, ChangeMarkerT>>(
      drawArgs, changeMarker, childrenNodes
    );
  }
};

template<typename ContextT>
auto updateGraph(Box<NodeI<ContextT>> graph, Box<VirtualNodeI<ContextT>> const &vGraph)
  -> Box<NodeI<ContextT>> {
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

  ) {
    // This node will remain the same, but update its children
    std::size_t index = 0;

    auto oldChildren = graph->children();
    auto vChildren   = vGraph->children();
    immer::vector<Box<NodeI<ContextT>>> newChildren;

    while (index < vChildren.size()) {
      if (index < oldChildren.size()) {
        // Patch the children together
        newChildren = newChildren.push_back(updateGraph(oldChildren[index], vChildren[index]));
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
struct Fragment {};
struct FragmentGPUData {};

inline auto updateGPU([[maybe_unused]] Fragment frag) -> std::shared_ptr<FragmentGPUData> {
  return std::make_shared<FragmentGPUData>();
}

template<typename ContextT>
void draw(
  [[maybe_unused]] std::shared_ptr<ContextT> const &ctx,
  [[maybe_unused]] std::shared_ptr<FragmentGPUData> const &data,
  [[maybe_unused]] Fragment const &node
) {}

//--------------------------------------------------------------------------
// Helper functions that allow various options
template<typename ContextT, typename DrawArgsT, typename ChangeMarkerT>
auto n(DrawArgsT val, ChangeMarkerT marker, Vector<Box<VirtualNodeI<ContextT>>> children)
  -> Box<VirtualNodeI<ContextT>> {
  return std::make_shared<VirtualNode<ContextT, DrawArgsT, ChangeMarkerT>>(val, marker, children);
}

// Specialization to allow for use without a marker.
struct NoChangeMarker {};
inline auto operator==(
  [[maybe_unused]] NoChangeMarker const &lhs,
  [[maybe_unused]] NoChangeMarker const &rhs
) -> bool {
  return true;
}

template<typename ContextT, typename DrawArgsT>
auto n(DrawArgsT val, Vector<Box<VirtualNodeI<ContextT>>> children) -> Box<VirtualNodeI<ContextT>> {
  return n<ContextT>(val, NoChangeMarker{}, children);
}

template<typename ContextT, typename DrawArgsT, typename ChangeMarkerT>
auto n(DrawArgsT val, ChangeMarkerT marker) -> Box<VirtualNodeI<ContextT>> {
  return n<ContextT>(val, marker, {});
}

template<typename ContextT, typename DrawArgsT>
auto n(DrawArgsT val) -> Box<VirtualNodeI<ContextT>> {
  return n<ContextT>(val, NoChangeMarker{}, {});
}

template<typename ContextT, typename ChangeMarkerT>
auto f(ChangeMarkerT marker, Vector<Box<VirtualNodeI<ContextT>>> children)
  -> Box<VirtualNodeI<ContextT>> {
  return n<ContextT>(Fragment{}, marker, children);
}

template<typename ContextT>
auto f(Vector<Box<VirtualNodeI<ContextT>>> children) -> Box<VirtualNodeI<ContextT>> {
  return n<ContextT>(Fragment{}, NoChangeMarker{}, std::move(children));
}

template<typename ContextT>
auto empty() -> Box<VirtualNodeI<ContextT>> {
  return n<ContextT, Fragment, NoChangeMarker>(Fragment{}, NoChangeMarker{}, {});
}

// A helper to define values that aren't used until needed.
template<typename T>
class LazyVal {
  using Generator = std::function<T()>;
  Generator m_generator;

public:
  explicit LazyVal(T val)
    : m_generator{[val] { return val; }} {}

  explicit LazyVal(Generator gen)
    : m_generator{[gen]() { return gen(); }} {}

  explicit operator T() { return m_generator(); }

  auto operator()() -> T { return m_generator(); }
};
}  // namespace vsg
