# node_helpers.robust_rpc

The RobustRPC framework is one of the key components of ``node_helpers``. Its key API
is the ``RobustRPCMixin``, which provides a robust approach to handling errors in service
and action calls by propagating error messages raised by the server and re-raising them
on the client side.
This documentation aims to help users understand and effectively use the RobustRPCMixin.

The Robust RPC framework is documented in [docs/](../../../../docs/robust_rpc.rst)