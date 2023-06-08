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
#include <concepts>
#include <memory>
#include <type_traits>
#include <typeinfo>
#include <utility>

#include <immer/algorithm.hpp>
#include <immer/vector.hpp>

namespace vsg {

template<typename T>
using Vector = immer::vector<T>;

// We can't use immer::box, because it assumes you can concretely instantiate
// the T and this doesn't work with our type erasure
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
auto draw(
    Box<ContextT> const &ctx,
    Placeholder const &placeHolder1,
    Placeholder const &placeHolder2
) -> Box<ContextT>;

// This concept determines that a particular T can be drawn
template<typename ContextT, typename ArgsT, typename GPUDataT>
concept DrawableNode = requires(Box<ContextT> ctx, ArgsT args, GPUDataT data) {
    { updateGPU(args) } -> std::convertible_to<GPUDataT>;
    { draw(ctx, data, args) } -> std::convertible_to<Box<ContextT>>;
};

// A sentinel value that means we'll use equality of the draw args to detect change.
struct NoChangeMarkerProvided {};
inline auto operator==(
    [[maybe_unused]] NoChangeMarkerProvided const &lhs,
    [[maybe_unused]] NoChangeMarkerProvided const &rhs
) -> bool {
    return false;
}

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

    virtual auto draw(Box<ContextT> const &ctx) const -> Box<ContextT>           = 0;
    [[nodiscard]] virtual auto markerAsAny() const -> std::any                   = 0;
    [[nodiscard]] virtual auto drawArgsAsAny() const -> std::any                 = 0;
    [[nodiscard]] virtual auto drawArgsType() const -> std::type_info const    & = 0;
    [[nodiscard]] virtual auto children() const -> Vector<Box<NodeI<ContextT>>>  = 0;
    virtual void updateDrawArgs(std::any newArgs)                                = 0;
    virtual void updateChildren(Vector<Box<NodeI<ContextT>>> const &newChildren) = 0;
};

//------------------------------------------------------------------------------
// Nodes are the concrete elements of the scene graph that will be rendered.
//
// They hold the original args used to create them as well as the gpu data created
// when they were uploaded to the GPU and the change marker that was specified
// when they were created. They also hold a set of children nodes that forms the tree.
template<typename ContextT, typename DrawArgsT, typename ChangeMarkerT>
struct Node : public NodeI<ContextT> {
  private:
    DrawArgsT m_drawArgs;
    using GPUData = decltype(updateGPU(m_drawArgs));
    GPUData m_gpuData;
    ChangeMarkerT m_changeMarker;

    // TODO: This makes the children completely generic, which means ONLY the
    // Context
    //       can be passed along, none of the draw_args_t. :(
    //       Can we fix that?
    Vector<Box<NodeI<ContextT>>> m_children;

  public:
    Node(DrawArgsT args, ChangeMarkerT marker, Vector<Box<NodeI<ContextT>>> inChildren)
        : m_drawArgs{args}
        , m_gpuData{updateGPU(args)}
        , m_changeMarker{marker}
        , m_children{std::move(inChildren)} {}

    // Rule of 5.
    ~Node() override                           = default;
    Node(const Node &)                         = default;
    Node(Node &&) noexcept                     = default;
    auto operator=(const Node &) -> Node     & = default;
    auto operator=(Node &&) noexcept -> Node & = default;

    // Provide the base interface elements for our types.
    [[nodiscard]] auto markerAsAny() const -> std::any override { return std::any(m_changeMarker); }
    [[nodiscard]] auto drawArgsAsAny() const -> std::any override { return std::any(m_drawArgs); }
    [[nodiscard]] auto drawArgsType() const -> std::type_info const & override {
        return typeid(DrawArgsT);
    }
    [[nodiscard]] auto children() const -> Vector<Box<NodeI<ContextT>>> override {
        return m_children;
    }

    auto draw(Box<ContextT> const &ctx) const -> Box<ContextT> override {
        using vsg::draw;
        Box<ContextT> newCtx = draw(ctx, m_gpuData, m_drawArgs);
        immer::for_each(m_children, [&](const Box<NodeI<ContextT>> &curNode) {
            curNode->draw(newCtx);
        });
        return newCtx;
    }

    void updateChildren(Vector<Box<NodeI<ContextT>>> const &newChildren) override {
        m_children = newChildren;
    }

    void updateDrawArgs(std::any newArgs) override {
        if (newArgs.type() != typeid(m_drawArgs)) { return; }
        m_drawArgs = std::any_cast<DrawArgsT>(newArgs);
        m_gpuData  = updateGPU(m_drawArgs);
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
    [[nodiscard]] virtual auto hasProvidedChangeMarker() const -> bool                        = 0;
    [[nodiscard]] virtual auto changeMarkerEquals(NodeI<ContextT> const &other) const -> bool = 0;
    [[nodiscard]] virtual auto drawArgsEquals(NodeI<ContextT> const &other) const -> bool     = 0;
    [[nodiscard]] virtual auto drawArgsType() const -> std::type_info const                 & = 0;
    [[nodiscard]] virtual auto createNode() const -> Box<NodeI<ContextT>>                     = 0;
};

template<typename ContextT, typename DrawArgsT, typename ChangeMarkerT>
struct VirtualNode : public VirtualNodeI<ContextT> {
  private:
    DrawArgsT m_drawArgs;
    ChangeMarkerT m_changeMarker;
    Vector<Box<VirtualNodeI<ContextT>>> m_children;

  public:
    VirtualNode(
        DrawArgsT args,
        ChangeMarkerT marker,
        Vector<Box<VirtualNodeI<ContextT>>> inChildren
    )
        : m_drawArgs{args}
        , m_changeMarker{marker}
        , m_children{std::move(inChildren)} {}

    // Rule of 5.
    ~VirtualNode() override                                  = default;
    VirtualNode(const VirtualNode &)                         = default;
    VirtualNode(VirtualNode &&) noexcept                     = default;
    auto operator=(const VirtualNode &) -> VirtualNode     & = default;
    auto operator=(VirtualNode &&) noexcept -> VirtualNode & = default;

    // Provide the base interface elements for our types.
    [[nodiscard]] auto markerAsAny() const -> std::any override { return std::any(m_changeMarker); }
    [[nodiscard]] auto drawArgsAsAny() const -> std::any override { return std::any(m_drawArgs); }
    [[nodiscard]] auto drawArgsType() const -> std::type_info const & override {
        return typeid(DrawArgsT);
    }
    [[nodiscard]] auto children() const -> Vector<Box<VirtualNodeI<ContextT>>> override {
        return m_children;
    }
    [[nodiscard]] auto hasProvidedChangeMarker() const -> bool override {
        return typeid(m_changeMarker) != typeid(NoChangeMarkerProvided);
    }
    [[nodiscard]] auto changeMarkerEquals(NodeI<ContextT> const &other) const -> bool override {
        auto otherChangeMarker = other.markerAsAny();
        if (otherChangeMarker.type() != typeid(m_changeMarker)) { return false; }
        return std::any_cast<ChangeMarkerT>(otherChangeMarker) == m_changeMarker;
    }

    [[nodiscard]] auto drawArgsEquals(NodeI<ContextT> const &other) const -> bool override {
        if constexpr (std::same_as<NoChangeMarkerProvided, ChangeMarkerT> && !std::equality_comparable<DrawArgsT>) {
            static_assert(
                std::equality_comparable<DrawArgsT>,
                "The Draw Args you provided must be comparable with the == operator or you must "
                "provide a change marker."
            );
        } else if constexpr (!std::same_as<NoChangeMarkerProvided, ChangeMarkerT> && !std::equality_comparable<DrawArgsT>) {
            // TODO: Consider adding a way for this to warn people that this is happening.
            return false;
        } else {
            // TODO: could save a copy here by using the any only AFTER the types are the same
            auto otherDrawArgs = other.drawArgsAsAny();
            if (otherDrawArgs.type() != typeid(m_drawArgs)) { return false; }
            return std::any_cast<DrawArgsT>(otherDrawArgs) == m_drawArgs;
        }
    }

    [[nodiscard]] auto createNode() const -> Box<NodeI<ContextT>> override {
        Vector<Box<NodeI<ContextT>>> childrenNodes;
        immer::for_each(m_children, [&](const Box<VirtualNodeI<ContextT>> &child) {
            childrenNodes = childrenNodes.push_back(child->createNode());
        });
        return std::make_shared<vsg::Node<ContextT, DrawArgsT, ChangeMarkerT>>(
            m_drawArgs, m_changeMarker, childrenNodes
        );
    }
};

template<typename ContextT>
auto updateGraph(Box<NodeI<ContextT>> graph, Box<VirtualNodeI<ContextT>> const &vGraph)
    -> Box<NodeI<ContextT>> {
    // If the current graph is empty, create it from scratch.
    if (!graph) { return vGraph->createNode(); }

    // Else see if the current node needs to update its GPUData.
    bool needsUpdate = vGraph->hasProvidedChangeMarker() ? !vGraph->changeMarkerEquals(*graph)
                                                         : !vGraph->drawArgsEquals(*graph);
    if (needsUpdate) {
        // If we need to update AND our type changed, just recreate this entire node
        if (graph->drawArgsType() != vGraph->drawArgsType()) { return vGraph->createNode(); }

        // Otherwise update in place
        graph->updateDrawArgs(vGraph->drawArgsAsAny());
    }

    // Regardless of the new/old state of this node, merge the children of the nodes.
    std::size_t index = 0;

    auto oldChildren = graph->children();
    auto vChildren   = vGraph->children();
    immer::vector<Box<NodeI<ContextT>>> newChildren;

    // First merge any possibly "same" children nodes together.
    while (index < vChildren.size() && index < oldChildren.size()) {
        // NOTE: Because we recurse into updateGraph again here, the child nodes GPU data
        //       may be updated
        newChildren = newChildren.push_back(updateGraph(oldChildren[index], vChildren[index]));
        ++index;
    }

    // Then create all new ones.
    while (index < vChildren.size()) {
        // Completely new child.
        newChildren = newChildren.push_back(vChildren[index]->createNode());
        ++index;
    }

    // Store the new children.
    graph->updateChildren(newChildren);

    return graph;
}

//--------------------------------------------------------------------------
// In HTML we'd have loads of nodes to use as "containers" here, we don't
// So we'll make a simple one.
struct Fragment {};
inline auto operator==([[maybe_unused]] Fragment const &lhs, [[maybe_unused]] Fragment const &rhs)
    -> bool {
    return false;
}
struct FragmentGPUData {};

inline auto updateGPU([[maybe_unused]] Fragment frag) -> Box<FragmentGPUData> {
    return std::make_shared<FragmentGPUData>();
}

template<typename ContextT>
auto draw(
    [[maybe_unused]] Box<ContextT> const &ctx,
    [[maybe_unused]] Box<FragmentGPUData> const &data,
    [[maybe_unused]] Fragment const &node
) -> Box<ContextT> {
    return ctx;
}

//--------------------------------------------------------------------------
// A simple wrapper around another set of draw args that prevents the draw
// call for this draw args if it's invisible without removing it from the
// graph, which means showing it again will be faster.
template<class DrawArgsT>
struct Visibility {
    bool visible = true;
    DrawArgsT drawArgs;
};
template<class DrawArgsT>
auto operator==(Visibility<DrawArgsT> const &lhs, Visibility<DrawArgsT> const &rhs) -> bool {
    return lhs.drawArgs == rhs.drawArgs;
}
template<class GPUDataT>
struct VisibilityGPUData {
    VisibilityGPUData(GPUDataT inData)
        : data{std::move(inData)} {}
    GPUDataT data;
};

template<class DrawArgsT>
inline auto updateGPU(Visibility<DrawArgsT> const &frag)
    -> Box<VisibilityGPUData<decltype(updateGPU(frag.drawArgs))>> {
    return std::make_shared<VisibilityGPUData<decltype(updateGPU(frag.drawArgs))>>(
        updateGPU(frag.drawArgs)
    );
}

template<class ContextT, class DrawArgsT, class GPUDataT>
auto draw(
    [[maybe_unused]] Box<ContextT> const &ctx,
    [[maybe_unused]] Box<VisibilityGPUData<GPUDataT>> const &data,
    [[maybe_unused]] Visibility<DrawArgsT> const &node
) -> Box<ContextT> {
    if (node.visible) { return draw(ctx, data->data, node.drawArgs); }
    return ctx;
}

template<class DrawArgsT>
auto visible(DrawArgsT const &t, bool visible = true) -> Visibility<DrawArgsT> {
    return Visibility<DrawArgsT>{.visible = visible, .drawArgs = t};
}

//--------------------------------------------------------------------------
// Helper functions that allow various options
template<typename ContextT, typename DrawArgsT, typename ChangeMarkerT>
auto n(DrawArgsT val, ChangeMarkerT marker, Vector<Box<VirtualNodeI<ContextT>>> children)
    -> Box<VirtualNodeI<ContextT>> {
    return std::make_shared<VirtualNode<ContextT, DrawArgsT, ChangeMarkerT>>(val, marker, children);
}

// Specialization to allow for use without a marker.
template<typename ContextT, typename DrawArgsT>
auto n(DrawArgsT val, Vector<Box<VirtualNodeI<ContextT>>> children) -> Box<VirtualNodeI<ContextT>> {
    return n<ContextT>(val, NoChangeMarkerProvided{}, children);
}

template<typename ContextT, typename DrawArgsT, typename ChangeMarkerT>
auto n(DrawArgsT val, ChangeMarkerT marker) -> Box<VirtualNodeI<ContextT>> {
    return n<ContextT>(val, marker, {});
}

template<typename ContextT, typename DrawArgsT>
auto n(DrawArgsT val) -> Box<VirtualNodeI<ContextT>> {
    return n<ContextT>(val, NoChangeMarkerProvided{}, {});
}

template<typename ContextT, typename ChangeMarkerT>
auto f(ChangeMarkerT marker, Vector<Box<VirtualNodeI<ContextT>>> children)
    -> Box<VirtualNodeI<ContextT>> {
    return n<ContextT>(Fragment{}, marker, children);
}

template<typename ContextT>
auto f(Vector<Box<VirtualNodeI<ContextT>>> children) -> Box<VirtualNodeI<ContextT>> {
    return n<ContextT>(Fragment{}, NoChangeMarkerProvided{}, std::move(children));
}

template<typename ContextT>
auto empty() -> Box<VirtualNodeI<ContextT>> {
    return n<ContextT, Fragment, NoChangeMarkerProvided>(Fragment{}, NoChangeMarkerProvided{}, {});
}

template<class ContextT>
class Helper {
  public:
    template<typename DrawArgsT, typename ChangeMarkerT>
    auto n(DrawArgsT val, ChangeMarkerT marker, Vector<Box<VirtualNodeI<ContextT>>> children)
        -> Box<VirtualNodeI<ContextT>> {
        return vsg::n(val, marker, children);
    }

    template<typename DrawArgsT>
    auto n(DrawArgsT val, Vector<Box<VirtualNodeI<ContextT>>> children)
        -> Box<VirtualNodeI<ContextT>> {
        return vsg::n<ContextT>(val, NoChangeMarkerProvided{}, children);
    }

    template<typename DrawArgsT, typename ChangeMarkerT>
    auto n(DrawArgsT val, ChangeMarkerT marker) -> Box<VirtualNodeI<ContextT>> {
        return vsg::n<ContextT>(val, marker, {});
    }

    template<typename DrawArgsT>
    auto n(DrawArgsT val) -> Box<VirtualNodeI<ContextT>> {
        return vsg::n<ContextT>(val, NoChangeMarkerProvided{}, {});
    }

    template<typename ChangeMarkerT>
    auto f(ChangeMarkerT marker, Vector<Box<VirtualNodeI<ContextT>>> children)
        -> Box<VirtualNodeI<ContextT>> {
        return vsg::n<ContextT>(Fragment{}, marker, children);
    }

    auto f(Vector<Box<VirtualNodeI<ContextT>>> children) -> Box<VirtualNodeI<ContextT>> {
        return vsg::n<ContextT>(Fragment{}, NoChangeMarkerProvided{}, std::move(children));
    }

    auto empty() -> Box<VirtualNodeI<ContextT>> {
        return vsg::n<ContextT, Fragment, NoChangeMarkerProvided>(
            Fragment{}, NoChangeMarkerProvided{}, {}
        );
    }
};

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
